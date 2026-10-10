#!/usr/bin/env python3
"""Snapshot GitHub's reach data into a git-tracked archive, and render it.

WHY THIS EXISTS: GitHub's traffic API is a ROLLING 14-DAY WINDOW and is not
retroactive. Everything older is discarded by GitHub and cannot be recovered by
anyone. A daily snapshot kept in the repo (the metrics-data branch) is the only way this project
ever has a history of its own reach. Release asset counts do not expire, but
they are CUMULATIVE totals with no per-period breakdown, so the only way to
learn "how many downloads last week" is to diff two snapshots -- which again
requires keeping them.

Collection runs DAILY (and on every release). Any cadence under a fortnight
observes every day -- each traffic call returns FOURTEEN daily buckets -- but
the margin is what matters: weekly left one run of slack, so a single failure
was already a deadline, while daily leaves thirteen. Merging is by date keeping
the max, so the heavy overlap between daily runs is idempotent and a
partially-elapsed day is corrected by the next run rather than frozen at its
partial value.

Daily rewrites cost archive size, so snapshot-keyed stores are THINNED: every
snapshot for 30 days, then one per ISO week. Counters only rise, so the last
snapshot of a week carries that week's maximum and `_release_rollup` takes the
max across survivors -- no lifetime total can change, only the resolution of
the old delta column. Measured: a year of daily snapshots thins 365 -> 80 with
identical totals.

TWO POPULATIONS, NEVER SUMMED. The PCM zip (KiCad's Plugin and Content Manager
fetches it on install/update) and the prebuilt `grid_router-*` binaries
(downloaded by `build_router.py`) measure different audiences: a PCM user may
never touch git, and a from-source user may never touch PCM. PCM installs also
pile up on whichever release PCM currently points at, so a newer release
looking "smaller" is usually PCM pointing elsewhere, not a collapse in interest.

WHAT THIS CANNOT TELL YOU, stated on the page for the same reason it is stated
here: a download is not a run, and none of these numbers can separate one
person routing daily from a hundred who installed once and bounced. Only
telemetry answers that, and this repo collects none.
"""
KRT_TOOL = {'scope': [], 'kind': 'instrument'}

import argparse
import json
import os
import re
import shutil
import subprocess
import sys
import tempfile
import urllib.error
import urllib.parse
import urllib.request
from datetime import datetime, timezone

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
#: The snapshot ARCHIVE. Overridable (--data-dir / KRT_METRICS_DATA) because it
#: does not live in this checkout in CI: the traffic API is a rolling 14-day
#: window GitHub never backfills, so the snapshots must accumulate somewhere
#: durable -- but that somewhere does not have to be `main`. The workflow
#: checks out the orphan `metrics-data` branch and points this at it, so the
#: daily bookkeeping commit lands there and main stays free of it.
DATA = os.environ.get('KRT_METRICS_DATA') or os.path.join(ROOT, 'metrics', 'data')
#: The rendered site. Regenerated every run and uploaded straight to Pages, so
#: it is gitignored and never committed.
SITE = os.environ.get('KRT_METRICS_SITE') or os.path.join(ROOT, 'docs', 'site')

#: The prebuilt router binaries `build_router.py` fetches, by platform label.
PLATFORMS = (
    ('grid_router-linux-x86_64.so', 'Linux x86_64'),
    ('grid_router-windows-x86_64.pyd', 'Windows x86_64'),
    ('grid_router-macos-arm64.so', 'macOS arm64'),
    ('grid_router-macos-x86_64.so', 'macOS x86_64'),
)
#: The PCM package: `KiCadRoutingTools-<version>.zip`.
_PCM_RE = re.compile(r'^KiCadRoutingTools-.*\.zip$')


def _today():
    return datetime.now(timezone.utc).strftime('%Y-%m-%d')


# --------------------------------------------------------------- the API


def _repo_slug(explicit=''):
    """owner/name, from --repo, the Actions env, or the git remote."""
    if explicit:
        return explicit
    if os.environ.get('GITHUB_REPOSITORY'):
        return os.environ['GITHUB_REPOSITORY']
    try:
        url = subprocess.run(['git', '-C', ROOT, 'remote', 'get-url', 'origin'],
                             capture_output=True, text=True).stdout.strip()
    except Exception:
        url = ''
    m = re.search(r'github\.com[:/](.+?)(?:\.git)?$', url)
    if not m:
        raise SystemExit('cannot determine the repo; pass --repo owner/name')
    return m.group(1)


def _api(slug, path, token='', paginate=False):
    """GET one API path. Returns (payload, error) -- never raises.

    A failing endpoint must not cost the ones that work: traffic needs PUSH
    access while releases are public, so a token without it should still bank
    the release history rather than losing the whole run. The error travels to
    the page, because a silently absent series looks exactly like a quiet week.

    `paginate` is NOT optional for a list endpoint, and the first CI run proved
    why: this path returned page 1 only -- 30 of 39 releases -- while the local
    `gh --paginate` fallback returned all of them. The two fronts silently
    disagreed, and the CI answer would have dropped the nine oldest releases
    (and their lifetime PCM installs) out of the archive, looking for all the
    world like a real decline.
    """
    tok = token or os.environ.get('GITHUB_TOKEN') or os.environ.get('GH_TOKEN') or ''
    if tok:
        try:
            out, page = [], 1
            while True:
                sep = '&' if '?' in path else '?'
                url = (f'https://api.github.com/repos/{slug}/{path}'
                       + (f'{sep}per_page=100&page={page}' if paginate else ''))
                req = urllib.request.Request(url, headers={
                    'Authorization': f'Bearer {tok}',
                    'Accept': 'application/vnd.github+json',
                    'User-Agent': 'KiCadRoutingTools-metrics',
                })
                with urllib.request.urlopen(req, timeout=30) as r:
                    chunk = json.loads(r.read().decode())
                if not paginate or not isinstance(chunk, list):
                    return chunk, ''
                out.extend(chunk)
                # A short page is the last page. Also stop on an absurd number
                # of pages rather than looping forever on a misbehaving API.
                if len(chunk) < 100 or page >= 50:
                    return out, ''
                page += 1
        except urllib.error.HTTPError as e:
            err = f'HTTP {e.code}'
        except Exception as e:                                # pragma: no cover
            err = str(e)
    else:
        err = 'no token in env'
    # Fall back to the gh CLI, which is how this runs on a developer machine
    # where the token lives in gh's own keyring and not in the environment.
    try:
        p = subprocess.run(['gh', 'api', f'repos/{slug}/{path}', '--paginate'],
                           capture_output=True, text=True, timeout=120)
        if p.returncode == 0:
            txt = p.stdout.strip()
            # --paginate concatenates JSON arrays as `][`; stitch them back.
            if '][' in txt:
                txt = '[' + txt.replace('][', ',').strip('[]') + ']'
            return json.loads(txt), ''
        err = f'{err}; gh: {p.stderr.strip()[:120]}'
    except Exception as e:
        err = f'{err}; gh: {e}'
    return None, err


# --------------------------------------------------------------- storage


def _load(name, default):
    p = os.path.join(DATA, name)
    if not os.path.isfile(p):
        return default
    try:
        with open(p) as f:
            return json.load(f)
    except Exception:
        return default


def _save(name, obj):
    os.makedirs(DATA, exist_ok=True)
    with open(os.path.join(DATA, name), 'w') as f:
        json.dump(obj, f, indent=1, sort_keys=True)
        f.write('\n')


def thin_snapshots(store, keep_days=30, today=None):
    """Keep every snapshot from the last `keep_days`, then one per ISO week.

    Daily collection rewrites a full ~12 KB release snapshot every day, which
    is ~4 MB a year and rising as the release list grows. Old snapshots are
    almost all redundant: a download counter only ever goes up, so the LAST
    snapshot of a week carries that week's maximum for every asset, and
    `_release_rollup` takes the max across whatever survives. Thinning
    therefore cannot change any lifetime total -- only the resolution of the
    per-snapshot delta column, which nobody reads at day granularity a year
    back.

    The newest snapshot is always kept, whatever else happens, because it is
    the one the page renders from.
    """
    from datetime import date as _date
    if today is None:
        today = _date(*map(int, _today().split('-')))
    stamps = sorted(store)
    if not stamps:
        return 0
    keep = {stamps[-1]}
    weekly = {}
    for s in stamps:
        try:
            d = _date(*map(int, s.split('-')))
        except Exception:
            keep.add(s)          # unparseable: never silently discard it
            continue
        if (today - d).days <= keep_days:
            keep.add(s)
        else:
            y, w, _ = d.isocalendar()
            weekly[(y, w)] = s   # last one wins, i.e. that week's maximum
    keep |= set(weekly.values())
    dropped = [s for s in stamps if s not in keep]
    for s in dropped:
        del store[s]
    return len(dropped)


def _merge_daily(store, key, rows):
    """Merge `rows` into store[key] by date, keeping the MAX per date.

    Max, not overwrite: the current day is observed part-elapsed and would
    otherwise be frozen low by whichever run happened to see it first, and a
    re-run inside the same window must never reduce a banked day.
    """
    dst = store.setdefault(key, {})
    added = 0
    for row in rows or []:
        day = str(row.get('timestamp', ''))[:10]
        if not day:
            continue
        cur = dst.get(day) or {'count': 0, 'uniques': 0}
        new = {'count': max(cur['count'], int(row.get('count', 0))),
               'uniques': max(cur['uniques'], int(row.get('uniques', 0)))}
        if new != cur:
            added += 1
        dst[day] = new
    return added


#: KiCad's PCM catalogue is GENERATED from this GitLab repo, so the merge that
#: added a version to our package file is when PCM began serving that version.
_PCM_UPSTREAM = 'https://gitlab.com/api/v4/projects/kicad%2Faddons%2Fmetadata'
_PCM_UPSTREAM_GIT = 'https://gitlab.com/kicad/addons/metadata.git'


def _pcm_file_commits(rel_path):
    """Upstream commits touching `rel_path`, newest first -> (list, error).

    Each is {'id', 'committed_date'}, the shape the API's commit listing had.
    Read from a blobless clone rather than the API: GitLab's Cloudflare front
    answers the anonymous `repository/commits` LISTING with a bot challenge
    (HTTP 403, an HTML "Just a moment..." page) while git over HTTPS and the
    per-commit endpoints `_gitlab` still uses answer normally. A path-limited
    log needs trees only, so no blob is downloaded. Never raises.
    """
    env = {**os.environ, 'GIT_TERMINAL_PROMPT': '0'}   # never wait on a login
    with tempfile.TemporaryDirectory() as tmp:
        try:
            p = subprocess.run(['git', 'clone', '-q', '--filter=blob:none', '--no-checkout',
                                '--single-branch', _PCM_UPSTREAM_GIT, tmp],
                               capture_output=True, text=True, timeout=300, env=env)
            if p.returncode:
                return None, f'git clone: {p.stderr.strip()[-160:]}'
            p = subprocess.run(['git', '-C', tmp, 'log', '--format=%H %cI', '--', rel_path],
                               capture_output=True, text=True, timeout=120, env=env)
            if p.returncode:
                return None, f'git log: {p.stderr.strip()[-160:]}'
        except Exception as e:
            return None, f'git: {e}'
    return [{'id': sha, 'committed_date': when}
            for sha, when in (ln.split() for ln in p.stdout.splitlines() if ln.strip())], ''


def _gitlab(path):
    """GET one public GitLab API path, anonymously. (payload, error), never raises."""
    try:
        req = urllib.request.Request(f'{_PCM_UPSTREAM}/{path}',
                                     headers={'User-Agent': 'KiCadRoutingTools-metrics'})
        with urllib.request.urlopen(req, timeout=30) as r:
            return json.loads(r.read().decode()), ''
    except urllib.error.HTTPError as e:
        return None, f'HTTP {e.code}'
    except Exception as e:                                    # pragma: no cover
        return None, str(e)


def _vkey(v):
    return tuple(int(x) if x.isdigit() else 0 for x in str(v).split('.'))


def collect_pcm_listings(store):
    """Bank WHEN the PCM catalogue began serving each version -> error or ''.

    GitHub records when a release was published, not when PCM started handing
    it out, and PCM serves only the newest listed version -- so an unlisted
    release gets almost no zip installs while the listed one before it keeps
    them all. The per-release timeline cannot be placed without this history.

    `store` is {commit sha: {'version', 'listed'}}, one entry per upstream
    commit that touched our package file. The version is the newest one in the
    FILE at that commit (an MR title can be stale; the file is what PCM reads),
    and the date is the MR's merge, falling back to the commit date for a
    direct push. Each sha is fetched once and kept: this is history, it never
    changes, and a failed fetch is simply retried by the next run.
    """
    try:
        with open(os.path.join(ROOT, 'metadata.json')) as f:
            ident = json.load(f)['identifier']
    except Exception as e:
        return f'metadata.json: {e}'
    rel = f'packages/{ident}/metadata.json'
    fpath = urllib.parse.quote(rel, safe='')
    commits, err = _pcm_file_commits(rel)
    if err:
        return err
    for c in commits:
        sha = c.get('id', '')
        if not sha or sha in store:
            continue
        doc, err = _gitlab(f'repository/files/{fpath}/raw?ref={sha}')
        versions = [v.get('version', '') for v in (doc or {}).get('versions') or []]
        if err or not versions:
            return err or f'{sha[:8]}: no versions in the package file'
        mrs, _err = _gitlab(f'repository/commits/{sha}/merge_requests')
        when = next((m['merged_at'] for m in mrs or [] if m.get('merged_at')),
                    c.get('committed_date', ''))
        try:
            listed = datetime.fromisoformat(when.replace('Z', '+00:00')) \
                .astimezone(timezone.utc).date().isoformat()
        except Exception:
            return f'{sha[:8]}: unreadable date {when!r}'
        store[sha] = {'version': max(versions, key=_vkey), 'listed': listed}
    return ''


def pcm_listing_dates(store):
    """{sha: {version, listed}} -> {release tag: first day PCM served it}."""
    out = {}
    for rec in (store or {}).values():
        tag, day = 'v' + str(rec.get('version', '')), str(rec.get('listed', ''))[:10]
        if len(tag) > 1 and day and (tag not in out or day < out[tag]):
            out[tag] = day
    return out


# --------------------------------------------------------------- collect


def collect(slug, token=''):
    stamp = _today()
    errors = {}
    collected = []

    traffic = _load('traffic_daily.json', {})
    for ep, key in (('traffic/views', 'views'), ('traffic/clones', 'clones')):
        payload, err = _api(slug, ep, token)
        if err or not isinstance(payload, dict):
            errors[ep] = err or 'unexpected payload'
            continue
        n = _merge_daily(traffic, key, payload.get(key))
        collected.append(ep)
        print(f'  {ep}: {n} day(s) new/updated')
    _save('traffic_daily.json', traffic)

    for ep, fname in (('traffic/popular/referrers', 'referrers.json'),
                      ('traffic/popular/paths', 'paths.json')):
        payload, err = _api(slug, ep, token)
        if err or not isinstance(payload, list):
            errors[ep] = err or 'unexpected payload'
            continue
        store = _load(fname, {})
        store[stamp] = payload
        n = thin_snapshots(store)
        _save(fname, store)
        collected.append(ep)
        if n:
            print(f'  {fname}: thinned {n} old snapshot(s) to weekly')
        print(f'  {ep}: {len(payload)} row(s) snapshotted')

    payload, err = _api(slug, 'releases', token, paginate=True)
    if err or not isinstance(payload, list):
        errors['releases'] = err or 'unexpected payload'
    else:
        store = _load('releases.json', {})
        snap = {}
        for rel in payload:
            snap[rel['tag_name']] = {
                'published_at': rel.get('published_at', ''),
                'assets': {a['name']: int(a.get('download_count', 0))
                           for a in rel.get('assets') or []},
            }
        store[stamp] = snap
        n = thin_snapshots(store)
        _save('releases.json', store)
        # WHEN, not just which day: a scheduled run lands anywhere from ~11:00
        # to ~13:00 UTC and a manual or release-triggered run replaces the
        # day's snapshot at any hour, so intervals between snapshots are not a
        # day long. `reign_downloads` spreads each one over the hours it spans.
        times = _load('release_times.json', {})
        times[stamp] = datetime.now(timezone.utc).isoformat(timespec='seconds')
        _save('release_times.json', {k: v for k, v in times.items() if k in store})
        collected.append('releases')
        if n:
            print(f'  releases.json: thinned {n} old snapshot(s) to weekly')
        print(f'  releases: {len(snap)} release(s) snapshotted')

    listings = _load('pcm_listings.json', {})
    err = collect_pcm_listings(listings)
    _save('pcm_listings.json', listings)   # whatever was banked before a failure
    if err:
        errors['pcm listings (gitlab kicad/addons/metadata)'] = err
    else:
        collected.append('pcm_listings')
        print(f'  pcm listings: {len(pcm_listing_dates(listings))} version(s) listed')

    meta = _load('meta.json', {})
    meta['last_collected'] = datetime.now(timezone.utc).isoformat(timespec='seconds')
    meta['repo'] = slug
    meta['errors'] = errors
    _save('meta.json', meta)
    if errors:
        print('  WARN endpoints that failed (the page will say so):')
        for k, v in errors.items():
            print(f'    {k}: {v}')
    return errors, collected


# --------------------------------------------------------------- render


def _esc(s):
    return (str(s).replace('&', '&amp;').replace('<', '&lt;')
            .replace('>', '&gt;').replace('"', '&quot;'))


def _axis_frame(svg, top, left_label, right_label, height=200):
    """Wrap a stretched plot in HTML axis labels.

    THE PROBLEM THIS SOLVES: the plot SVG is `width:100%` with
    `preserveAspectRatio="none"`, which is right for the GEOMETRY -- the series
    should fill whatever width the page has -- but it scales the SVG's internal
    coordinate system, so any <text> inside scales and stretches with it. Axis
    numbers ended up huge on a wide window and unreadable on a narrow one, in a
    font nobody chose.

    So no text goes in the SVG at all. The tick values and end dates are HTML,
    positioned around the plot, and therefore sit at a real CSS font size at
    every width. Strokes carry `vector-effect="non-scaling-stroke"` for the same
    reason: a 2px line must stay 2px, not stretch with the viewBox.
    """
    ticks = ''.join(
        f'<span class="yl" style="top:{(1 - frac) * 100:.0f}%">'
        f'{_fmt_tick(top * frac)}</span>' for frac in (1, 0.5, 0))
    return (f'<div class="plotbox" style="--ph:{height}px">{ticks}'
            f'<div class="plot">{svg}</div></div>'
            f'<div class="xaxis"><span>{_esc(left_label)}</span>'
            f'<span>{_esc(right_label)}</span></div>')


def _fmt_tick(v):
    """Axis numbers stay short so a narrow gutter never truncates them."""
    v = float(v)
    if v >= 1000:
        return f'{v / 1000:.1f}k'.replace('.0k', 'k')
    if v >= 10 or v == int(v):
        return f'{int(round(v))}'
    return f'{v:.1f}'


def _legend(series, mark=None):
    keys = [f'<span class="key"><i class="ds" style="border-color:{s["color"]}"></i>'
            if s.get('dash') else
            f'<span class="key"><i style="background:{s["color"]}"></i>'
            for s in series]
    keys = [k + f'{_esc(s["label"])}</span>' for k, s in zip(keys, series)]
    if mark:
        keys.append(f'<span class="key"><i class="mk"></i>{_esc(mark[1])}</span>')
    return '<div class="legend">' + ' '.join(keys) + '</div>'


def _line_chart(series, width=880, height=200, mark=None):
    """Inline SVG, no dependencies -- the page must render from a file:// URL.

    `mark` = (day, label) draws a dashed vertical rule at that day, labelled in
    the legend (never in the SVG -- see `_axis_frame`). A series with `dash`
    is drawn dashed, and its legend key is a dashed stroke to match.
    """
    days = sorted(set().union(*[set(s['points']) for s in series if s['points']])
                  or {''})
    days = [d for d in days if d]
    if not days:
        return '<p class="muted">no data yet</p>'
    top = max([max(s['points'].values() or [0]) for s in series] + [1])
    pad = 3.0
    w, h = width, height - 2 * pad

    def xy(i, v):
        x = w * i / max(1, len(days) - 1)
        y = pad + h - (h * v / top)
        return f'{x:.1f},{y:.1f}'

    out = [f'<svg viewBox="0 0 {width} {height}" class="chart" '
           f'preserveAspectRatio="none" role="img">']
    for frac in (0, 0.5, 1):
        y = pad + h - h * frac
        out.append(f'<line x1="0" y1="{y:.1f}" x2="{w}" y2="{y:.1f}" '
                   f'class="grid" vector-effect="non-scaling-stroke"/>')
    for s in series:
        pts = ' '.join(xy(i, s['points'].get(d, 0)) for i, d in enumerate(days))
        out.append(f'<polyline points="{pts}" fill="none" stroke="{s["color"]}" '
                   f'stroke-width="2" stroke-linejoin="round" '
                   f'{"stroke-dasharray=\"6 4\" " if s.get("dash") else ""}'
                   f'vector-effect="non-scaling-stroke"/>')
    if mark and mark[0] in days:
        x = xy(days.index(mark[0]), 0).split(',')[0]
        out.append(f'<line x1="{x}" y1="0" x2="{x}" y2="{height}" class="mark" '
                   f'vector-effect="non-scaling-stroke"/>')
    else:
        mark = None
    out.append('</svg>')
    return (_axis_frame(''.join(out), top, days[0], days[-1], height)
            + _legend(series, mark))


def _zip_series(tag, listed):
    """Which series a release's ZIP downloads belong to: 'pcm' or 'zip'.

    PCM fetches only the zips of versions its catalogue lists, so a zip from a
    release it never listed cannot be a PCM install -- it was downloaded from
    the release page, or by automation (v0.19.0, never listed and superseded
    for two months, took 415 zip downloads in a day). Those are DIRECT zip
    downloads. With no listing history at all nothing can be told apart, and
    every zip counts as PCM, which the page then says it cannot vouch for.
    """
    return 'pcm' if not listed or tag in listed else 'zip'


def reign_downloads(releases, listed=None, times=None, partial=False):
    """Snapshots -> (downloads-per-day timeline, first MEASURED day or None).

    Three series, never summed: 'pcm' (zips of PCM-listed releases), 'zip'
    (direct downloads of every other zip, see `_zip_series`) and 'bin' (the
    router binaries).

    WHY NOT BARS. A release's counter is cumulative and never stops rising, and
    PCM piles every install onto whichever release it serves -- so a bar chart
    shows a few towers and many stubs, and says nothing about WHEN any of it
    happened.

    MEASURED where the archive can measure it. From the first snapshot on, the
    difference between two snapshots IS the downloads of that interval, with no
    assumption at all. Each interval is spread over the HOURS it covers, from
    `times` ({snapshot key: ISO time}, recorded by the collector). Booking a
    whole interval to one day was wrong whenever a run came late: a manual run
    at 19:04 made the last interval 31.7 hours long and put all of it on one
    day. A snapshot with no recorded time counts from the start of its day.
    The day the last snapshot falls in is still accumulating, so it is left out
    unless `partial` -- a part-day drawn as a day always reads as a collapse.

    ESTIMATED only before the first snapshot, where each release's count is one
    lifetime total. It is spread evenly over the release's REIGN: from when it
    became the newest release until its successor did. For the zip of a release
    PCM listed, both ends are PCM listing dates (`listed`, {tag: day}, from
    `collect_pcm_listings`), because PCM serves only its newest listed version
    and skips every release in between. The archive measured why this is the
    right window: the day PCM switched from v0.20.4 to v0.22.1, v0.20.4 fell
    from ~150 installs a day to ~3, and superseded binaries likewise drop to
    about one a day as soon as the next release is published.

    The window used to run from publish to TODAY instead, and that made the
    total slope upward whatever interest did: every release ever published kept
    contributing to every later day, so the sum grew with the size of the
    catalogue. With reigns, a flat level of interest draws a flat line.

    Two biases remain, and the page states both: the trickle a superseded
    release keeps gathering is booked inside its own reign, so the earliest
    plateaus read slightly high; and even spread flattens the burst right after
    each release. Neither creates or destroys a download -- with `partial` the
    timeline sums to the same max-across-snapshots totals `_release_rollup`
    reports.
    """
    from datetime import date as _date, datetime as _dt, time as _time, timedelta
    one = timedelta(days=1)
    zero = {'pcm': 0.0, 'zip': 0.0, 'bin': 0.0}

    def day(s):
        try:
            return _date(*map(int, str(s)[:10].split('-')))
        except Exception:
            return None

    def midnight(d):
        return _dt.combine(d, _time())

    def at(stamp):
        """When a snapshot was taken: its recorded time, else its day's start."""
        try:
            t = _dt.fromisoformat(str((times or {}).get(stamp, '')).replace('Z', '+00:00'))
            if t.tzinfo:
                t = t.astimezone(timezone.utc).replace(tzinfo=None)
        except ValueError:
            t = None
        # The key IS the collection date, so a time on another day is a
        # corrupt record, not a fact to honour.
        return t if t and t.date() == day(stamp) else midnight(day(stamp))

    stamps = [s for s in sorted(releases) if day(s)]
    if not stamps:
        return {}, None
    first = at(stamps[0])
    pub = {}
    for s in stamps:
        for tag, rel in (releases[s] or {}).items():
            if day(rel.get('published_at')):
                pub[tag] = rel['published_at']
    started = {t: day(p) for t, p in pub.items()}
    on_pcm = {t: day(d) for t, d in (listed or {}).items() if t in pub and day(d)}
    names = {n for n, _ in PLATFORMS}
    kinds = (('zip', lambda k: bool(_PCM_RE.match(k))), ('bin', lambda k: k in names))

    out = {}

    def book(t0, t1, mass, key):
        """Spread `mass` evenly over the instants [t0, t1), day by day."""
        span = (t1 - t0).total_seconds()
        if mass <= 0 or span <= 0:
            return
        d = t0.date()
        while midnight(d) < t1:
            lo, hi = max(t0, midnight(d)), min(t1, midnight(d + one))
            cell = out.setdefault(d.isoformat(), dict(zero))
            cell[key] += mass * (hi - lo).total_seconds() / span
            d += one

    def successor(start, starts):
        later = [d for d in starts if d > start]
        return min(later) if later else None

    for tag in sorted(pub, key=lambda t: pub[t]):
        for kind, pred in kinds:
            key = _zip_series(tag, listed) if kind == 'zip' else 'bin'
            cum, peak = [], 0
            for s in stamps:
                assets = ((releases[s] or {}).get(tag) or {}).get('assets') or {}
                # Running max, the same discipline as `_release_rollup`: a short
                # read must not become a negative day followed by a double one.
                peak = max(peak, sum(int(v) for k, v in assets.items() if pred(k)))
                cum.append(peak)
            if key == 'pcm' and tag in on_pcm:
                start = on_pcm[tag]
                end = successor(start, on_pcm.values())
            else:
                start = started[tag]
                end = successor(start, started.values())
            # Before the archive: the reign, cut at the first snapshot, and
            # never shorter than the release's own first day.
            lo = midnight(start)
            hi = min(midnight(end), first) if end else first
            book(lo, hi if hi > lo else lo + one, cum[0], key)
            for i in range(1, len(stamps)):
                book(at(stamps[i - 1]), at(stamps[i]), cum[i] - cum[i - 1], key)
    if not partial:
        out = {d: v for d, v in out.items() if d < stamps[-1][:10]}
    # Every day present, zeros included: the chart places points by INDEX, so
    # a day with no downloads left out would silently squeeze the time axis.
    if out:
        lo, hi = day(min(out)), day(max(out))
        for i in range((hi - lo).days + 1):
            out.setdefault((lo + i * one).isoformat(), dict(zero))
    return out, (stamps[0] if len(stamps) > 1 else None)


def _grouped_bars(rows, series, width=880, height=220):
    """Two bars per category. Labels live in HTML -- see `_axis_frame`."""
    if not rows:
        return '<p class="muted">no data yet</p>'
    top = max([v for _, d in rows for v in d.values()] + [1])
    pad = 3.0
    w, h = width, height - 2 * pad
    slot = w / max(1, len(rows))
    bw = max(1.5, slot / (len(series) + 1))
    out = [f'<svg viewBox="0 0 {width} {height}" class="chart" '
           f'preserveAspectRatio="none" role="img">']
    for frac in (0, 0.5, 1):
        y = pad + h - h * frac
        out.append(f'<line x1="0" y1="{y:.1f}" x2="{w}" y2="{y:.1f}" '
                   f'class="grid" vector-effect="non-scaling-stroke"/>')
    for i, (label, vals) in enumerate(rows):
        base = i * slot + (slot - bw * len(series)) / 2
        for j, (key, colour, sname) in enumerate(series):
            v = vals.get(key, 0)
            bh = h * v / top
            out.append(
                f'<rect x="{base + j * bw:.2f}" y="{pad + h - bh:.1f}" '
                f'width="{bw * 0.9:.2f}" height="{max(bh, 0.6):.1f}" fill="{colour}">'
                f'<title>{_esc(label)} — {_esc(sname)}: {v:,}</title></rect>')
    out.append('</svg>')
    return (_axis_frame(''.join(out), top, rows[0][0], rows[-1][0], height)
            + _legend([{'color': c, 'label': n} for _k, c, n in series]))


def _bars(rows, unit=''):
    if not rows:
        return '<p class="muted">no data yet</p>'
    top = max(v for _, v in rows) or 1
    out = ['<table class="bars">']
    for label, v in rows:
        out.append(f'<tr><th>{_esc(label)}</th><td class="barcell">'
                   f'<span class="bar" style="width:{100.0 * v / top:.1f}%"></span>'
                   f'</td><td class="num">{v:,}{unit}</td></tr>')
    out.append('</table>')
    return ''.join(out)


def weekly_rollup(traffic):
    """ISO-week sums with week-over-week change, newest first.

    THE TRAP THIS EXISTS TO DEFUSE: the current week is always PARTIAL, so it
    always looks like a collapse. A monitoring page that does not say which row
    is incomplete trains its reader to ignore the only signal it has -- or to
    panic every Monday. `days` is the observed day count and `partial` is True
    below seven, and the week-over-week figure is withheld (None) rather than
    computed against a stub.

    Sums of daily UNIQUES are reported as unique-days, never as people: the
    same cloner on Tuesday and Friday is two. That is what GitHub gives.
    """
    from datetime import date as _date
    weeks = {}
    for kind in ('clones', 'views'):
        for day, v in (traffic.get(kind) or {}).items():
            try:
                y, w, _ = _date(*map(int, day.split('-'))).isocalendar()
            except Exception:
                continue
            row = weeks.setdefault(f'{y}-W{w:02d}', {'week': f'{y}-W{w:02d}',
                                                     'days': set()})
            row['days'].add(day)
            row[kind] = row.get(kind, 0) + v.get('count', 0)
            row[kind + '_u'] = row.get(kind + '_u', 0) + v.get('uniques', 0)
    rows = []
    for key in sorted(weeks):
        r = weeks[key]
        r['days'] = len(r['days'])
        r['partial'] = r['days'] < 7
        for k in ('clones', 'views', 'clones_u', 'views_u'):
            r.setdefault(k, 0)
        rows.append(r)
    for i, r in enumerate(rows):
        prev = rows[i - 1] if i else None
        # No WoW against a partial week in EITHER position: comparing a stub
        # forwards or backwards manufactures a swing that is pure calendar.
        r['wow'] = (None if not prev or prev['partial'] or r['partial']
                    else r['clones'] - prev['clones'])
    return list(reversed(rows))


def clone_character(traffic, releases):
    """Per-day clones, unique cloners, and clones-per-unique.

    THE QUESTION THIS ANSWERS BADLY, AND WHY IT IS STILL THE BEST AVAILABLE.
    GitHub's clones endpoint returns a count and a unique count. No user agent,
    no IP, no actor -- so "was that a person?" cannot be answered, only
    estimated, and any page claiming a true human count is lying.

    `uniques` is the closest proxy, because a person clones once or twice while
    an automated fetcher clones repeatedly from few addresses: the RATIO
    count/uniques is therefore an automation index, low (~1.4) on ordinary days
    and high on machine days. Measured here, 2026-09-04 -- the v0.22.0 release
    day -- ran 528 clones at 4.26 per unique while VIEWS stayed flat at 281,
    against a 1.4-1.9 baseline.

    That day is also the measurement that killed the obvious refinement.
    Subtracting this project's own CI looks principled, and is worthless: the
    release run had SEVEN jobs, so ~7 checkouts against a ~300-clone excess.
    The spike is other people's machines -- downstream CI, mirrors, release
    trackers -- which no API here can identify. So the ratio is disclosed and
    left uncorrected rather than adjusted by a number that explains 1% of it.
    """
    clones = traffic.get('clones', {})
    views = traffic.get('views', {})
    rel_days = set()
    for snap in releases.values():
        for rel in snap.values():
            if rel.get('published_at'):
                rel_days.add(rel['published_at'][:10])
    rows = []
    for day in sorted(clones)[-14:]:
        c = clones[day]
        ratio = c['count'] / max(1, c['uniques'])
        rows.append({'date': day, 'count': c['count'], 'uniques': c['uniques'],
                     'ratio': ratio, 'views': views.get(day, {}).get('count', 0),
                     'release': day in rel_days})
    return rows


def _release_rollup(releases):
    """Latest snapshot -> per-release PCM / binary / total counts, newest first."""
    if not releases:
        return [], {}, {}
    # MAX ACROSS SNAPSHOTS, not the latest snapshot. A download counter only
    # ever grows, so the largest value seen is the true one -- and reading the
    # latest snapshot alone lets ONE short read (a rate limit mid-pagination, a
    # token change, the un-paginated bug the first CI run shipped) silently cut
    # the lifetime total and render it as a decline. Same discipline as the
    # traffic merge, for the same reason. A deleted release keeps its last
    # known counts, which is honest: those downloads did happen.
    latest = {}
    for stamp in sorted(releases):
        for tag, rel in (releases[stamp] or {}).items():
            cur = latest.setdefault(tag, {'published_at': rel.get('published_at', ''),
                                          'assets': {}})
            if rel.get('published_at'):
                cur['published_at'] = rel['published_at']
            for name, n in (rel.get('assets') or {}).items():
                cur['assets'][name] = max(cur['assets'].get(name, 0), int(n))
    rows, plat_tot, pcm_tot = [], {}, {}
    for tag, rel in latest.items():
        assets = rel.get('assets', {})
        pcm = sum(v for k, v in assets.items() if _PCM_RE.match(k))
        binaries = {}
        for name, label in PLATFORMS:
            if name in assets:
                binaries[label] = assets[name]
                plat_tot[label] = plat_tot.get(label, 0) + assets[name]
        if pcm:
            pcm_tot[tag] = pcm
        rows.append({'tag': tag, 'published': (rel.get('published_at') or '')[:10],
                     'pcm': pcm, 'binaries': binaries,
                     'total': sum(assets.values())})
    rows.sort(key=lambda r: r['published'], reverse=True)
    return rows, plat_tot, pcm_tot


def currently_accumulating(releases):
    """Which release is gaining PCM installs RIGHT NOW -> (tag, gain, since).

    "The release with the most installs" is a proxy for "the release PCM serves"
    and it goes stale in a known way: when PCM is repointed, the new release
    starts at zero while the old one keeps the larger lifetime total for months.
    A reader would keep being told about a release nobody is installing any more.

    The delta between the two most recent snapshots measures it directly --
    whichever release is still climbing is the one being served. That needs two
    snapshots on different DAYS (same-day runs share a key and overwrite), so it
    returns None until the archive has them, and the caller falls back to the
    lifetime maximum rather than inventing an answer.
    """
    stamps = sorted(releases)
    if len(stamps) < 2:
        return None
    prev, cur = releases[stamps[-2]], releases[stamps[-1]]
    gains = {tag: _pcm_count(cur, tag) - _pcm_count(prev, tag) for tag in cur}
    tag, gain = max(gains.items(), key=lambda kv: kv[1], default=(None, 0))
    if not tag or gain <= 0:
        return None
    return tag, gain, stamps[-2]


def _pcm_count(snap, tag):
    return sum(v for k, v in ((snap or {}).get(tag, {}).get('assets') or {}).items()
               if _PCM_RE.match(k))


def pcm_now_serving(store):
    """The release tag PCM's catalogue serves now, or None with no history.

    The NEWEST upstream commit decides, not the highest version ever listed:
    its file is what the catalogue is generated from, so a withdrawn version
    stops being named the moment the file stops listing it.
    """
    recs = [r for r in (store or {}).values() if r.get('version') and r.get('listed')]
    if not recs:
        return None
    return 'v' + max(recs, key=lambda r: (r['listed'], _vkey(r['version'])))['version']


def pcm_card_hint(releases, store, pcm_tot):
    """The PCM card's second line: the release PCM is serving, best source first.

    1. The listing history, when collected. That is a FACT about what the
       catalogue offers, so a burst on some other release cannot move it.
    2. Else the release that climbed most between the last two snapshots
       (`currently_accumulating`). Measured, but it is a guess about PCM, and
       it guesses wrong on a burst: v0.19.0 took 264 zip downloads in one day
       while PCM was serving v0.22.1, and the card named v0.19.0.
    3. Else the lifetime maximum, labelled as exactly that.
    """
    serving = pcm_now_serving(store)
    stamps = sorted(releases)
    if serving:
        if len(stamps) < 2:
            since = pcm_listing_dates(store).get(serving, '?')
            return f'now serving {serving}, listed {since}'
        gain = (_pcm_count(releases[stamps[-1]], serving)
                - _pcm_count(releases[stamps[-2]], serving))
        return f'now serving {serving} (+{max(0, gain):,} since {stamps[-2]})'
    acc = currently_accumulating(releases)
    if acc:
        return f'now serving {acc[0]} (+{acc[1]:,} since {acc[2]})'
    head = max(pcm_tot.items(), key=lambda kv: kv[1]) if pcm_tot else ('-', 0)
    return f'largest single release: {head[0]} ({head[1]:,})'


def _weekly_deltas(releases, listed=None):
    """Per-snapshot NEW downloads: cumulative counters differenced in time.

    Zips split the same way as the chart (`_zip_series`): PCM-listed releases
    in 'pcm', every other zip in 'zip'.
    """
    stamps = sorted(releases)
    names = {n for n, _ in PLATFORMS}
    preds = (('pcm', lambda t, k: bool(_PCM_RE.match(k)) and _zip_series(t, listed) == 'pcm'),
             ('zip', lambda t, k: bool(_PCM_RE.match(k)) and _zip_series(t, listed) == 'zip'),
             ('bin', lambda t, k: k in names))
    out = []
    for prev, cur in zip(stamps, stamps[1:]):
        a, b = releases[prev], releases[cur]

        def total(snap, pred):
            return sum(v for tag, rel in snap.items()
                       for k, v in rel.get('assets', {}).items() if pred(tag, k))
        row = {'date': cur}
        for key, pred in preds:
            row[key] = max(0, total(b, pred) - total(a, pred))
        out.append(row)
    return out


def render(slug):
    traffic = _load('traffic_daily.json', {})
    releases = _load('releases.json', {})
    referrers = _load('referrers.json', {})
    meta = _load('meta.json', {})

    rows, plat_tot, pcm_tot = _release_rollup(releases)
    listings = _load('pcm_listings.json', {})
    listed = pcm_listing_dates(listings)
    deltas = _weekly_deltas(releases, listed)
    views, clones = traffic.get('views', {}), traffic.get('clones', {})
    last14 = sorted(views)[-14:]

    def _sum(d, days, key):
        return sum(d.get(x, {}).get(key, 0) for x in days)

    pcm_hint = pcm_card_hint(releases, listings, pcm_tot)
    pcm_installs = sum(v for t, v in pcm_tot.items() if _zip_series(t, listed) == 'pcm')
    cards = [
        ('PCM installs', f'{pcm_installs:,}', pcm_hint),
        ('Router binaries', f'{sum(plat_tot.values()):,}',
         'prebuilt .so/.pyd, all releases'),
        ('Clones / 14d', f'{_sum(clones, last14, "count"):,}',
         f'{_sum(clones, last14, "uniques"):,} unique-days'),
        ('Views / 14d', f'{_sum(views, last14, "count"):,}',
         f'{_sum(views, last14, "uniques"):,} unique-days'),
    ]

    ref_rows = []
    if referrers:
        for r in referrers[max(referrers)][:12]:
            ref_rows.append((r.get('referrer', '?'), int(r.get('count', 0))))

    rel_html = []
    for r in rows[:14]:
        b = ' / '.join(f'{v:,}' for v in r['binaries'].values()) or '-'
        rel_html.append(
            f'<tr><th>{_esc(r["tag"])}'
            f'{"<span class=part> PCM</span>" if r["tag"] in listed else ""}</th>'
            f'<td>{_esc(r["published"])}</td>'
            f'<td class="num">{r["pcm"]:,}</td><td class="num">{b}</td>'
            f'<td class="num">{r["total"]:,}</td></tr>')

    delta_html = ''.join(
        f'<tr><th>{_esc(d["date"])}</th><td class="num">{d["pcm"]:,}</td>'
        f'<td class="num">{format(d["zip"], ",") if listed else "—"}</td>'
        f'<td class="num">{d["bin"]:,}</td></tr>' for d in deltas[-12:])

    errs = meta.get('errors') or {}
    err_html = ''
    if errs:
        items = ''.join(f'<li><code>{_esc(k)}</code> — {_esc(v)}</li>'
                        for k, v in errs.items())
        err_html = (f'<div class="warn"><strong>Incomplete collection.</strong> '
                    f'These endpoints failed, so their series has a hole rather '
                    f'than a quiet week:<ul>{items}</ul>'
                    f'The traffic endpoints need a token with push access.</div>')

    spread, measured_from = reign_downloads(
        releases, listed, _load('release_times.json', {}))
    dl_series = [
        {'label': 'PCM installs/day', 'color': '#3b82f6',
         'points': {d: v['pcm'] for d, v in spread.items()}},
        {'label': 'router binaries/day', 'color': '#f97316',
         'points': {d: v['bin'] for d, v in spread.items()}},
    ]
    if listed:
        # Dashed as well as a third hue: aqua sits close to the blue for a
        # tritan reader (ΔE 6.2), so the dash carries identity on its own.
        dl_series.append({'label': 'direct zip downloads/day', 'color': '#1baf7a',
                          'dash': True,
                          'points': {d: v['zip'] for d, v in spread.items()}})
    dl_chart = _line_chart(dl_series, height=220, mark=measured_from and (
        measured_from, f'measured from {measured_from}; estimated before'))
    if measured_from:
        dl_head = (f'Estimated before {_esc(measured_from)}, measured from '
                   f'then on.')
        dl_measured = (f'From {_esc(measured_from)} on, every point is the '
                       f'difference between two snapshots, spread over the '
                       f'hours between them, with no assumption at all; '
                       f'today is left out until it is over, because a part-day '
                       f'always looks like a collapse.')
    else:
        dl_head = 'Estimated rate, not a measurement — yet.'
        dl_measured = ('Once the archive holds two snapshots, the days after '
                       'the first are measured directly instead.')
    dl_split = (
        ' <strong>The PCM line counts only the zips of releases PCM '
        'listed.</strong> Every other zip is on the dashed line as a direct '
        'download, because PCM cannot install a version it does not list: '
        'those come from the release page, or from automation (a release '
        'superseded two months earlier once took 415 zip downloads in a day, '
        'with its Linux binary climbing alongside and nothing else).'
        if listed else
        ' <strong>The PCM listing history could not be read</strong>, so the '
        'PCM line counts every zip and treats every release as served by PCM '
        'in turn, neither of which is true: it is unreliable until a '
        'collection reaches GitLab.')

    wk = weekly_rollup(traffic)
    _wk_rows = []
    for r in wk:
        part = (f'<span class="part"> {r["days"]}/7 days</span>'
                if r['partial'] else '')
        wow = '—' if r['wow'] is None else format(r['wow'], '+,')
        _wk_rows.append(
            f'<tr><th>{_esc(r["week"])}{part}</th>'
            f'<td class="num">{r["clones"]:,}</td>'
            f'<td class="num">{r["clones_u"]:,}</td>'
            f'<td class="num">{wow}</td>'
            f'<td class="num">{r["views"]:,}</td></tr>')
    wk_html = ''.join(_wk_rows)
    lifetime_c = sum(v.get('count', 0) for v in traffic.get('clones', {}).values())
    lifetime_v = sum(v.get('count', 0) for v in traffic.get('views', {}).values())
    banked = len(traffic.get('clones', {}))

    char_rows = clone_character(traffic, releases)
    char_html = ''.join(
        f'<tr><th>{_esc(r["date"])}{" ●" if r["release"] else ""}</th>'
        f'<td class="num">{r["count"]:,}</td><td class="num">{r["uniques"]:,}</td>'
        f'<td class="num{" hot" if r["ratio"] >= 3 else ""}">{r["ratio"]:.2f}</td>'
        f'<td class="num">{r["views"]:,}</td></tr>' for r in char_rows)
    peak = max(char_rows, key=lambda r: r['ratio']) if char_rows else None

    chart = _line_chart([
        {'label': 'views', 'points': {d: v['count'] for d, v in views.items()},
         'color': '#3b82f6'},
        {'label': 'clones', 'points': {d: v['count'] for d, v in clones.items()},
         'color': '#f97316'},
    ])

    html = f"""<!doctype html>
<meta charset="utf-8">
<meta name="viewport" content="width=device-width,initial-scale=1">
<title>KiCadRoutingTools — reach</title>
<style>
:root {{ color-scheme: light dark;
  --bg:#fbfbfa; --fg:#1a1a18; --muted:#6b6b66; --line:#e2e2dd; --card:#fff; --warn:#fff7ed; }}
@media (prefers-color-scheme: dark) {{ :root {{
  --bg:#16171a; --fg:#e8e8e4; --muted:#9a9a94; --line:#2c2e33; --card:#1d1e22; --warn:#2a2118; }} }}
* {{ box-sizing:border-box }}
body {{ margin:0; padding:24px 16px 64px; background:var(--bg); color:var(--fg);
  font:15px/1.55 -apple-system,BlinkMacSystemFont,"Segoe UI",Roboto,sans-serif; }}
main {{ max-width:960px; margin:0 auto }}
h1 {{ font-size:1.5rem; margin:0 0 4px }}
h2 {{ font-size:1.05rem; margin:36px 0 10px; letter-spacing:.01em }}
.sub {{ color:var(--muted); margin:0 0 24px; font-size:.9rem }}
.cards {{ display:grid; grid-template-columns:repeat(auto-fit,minmax(180px,1fr)); gap:12px }}
.card {{ background:var(--card); border:1px solid var(--line); border-radius:10px; padding:14px 16px }}
.card .n {{ font-size:1.7rem; font-weight:600; letter-spacing:-.02em }}
.card .l {{ font-size:.78rem; text-transform:uppercase; letter-spacing:.06em; color:var(--muted) }}
.card .h {{ font-size:.78rem; color:var(--muted); margin-top:4px }}
table {{ border-collapse:collapse; width:100%; font-size:.9rem }}
th, td {{ text-align:left; padding:6px 10px; border-bottom:1px solid var(--line) }}
th {{ font-weight:600 }}
.num {{ text-align:right; font-variant-numeric:tabular-nums }}
.wrap {{ overflow-x:auto }}
.bars th {{ width:34%; font-weight:400 }}
.barcell {{ width:50% }}
.bar {{ display:block; height:9px; border-radius:3px; background:#3b82f6; min-width:2px }}
.plotbox {{ position:relative; padding-left:42px }}
.plotbox .yl {{ position:absolute; left:0; width:36px; text-align:right;
  font-size:11px; line-height:1; color:var(--muted); transform:translateY(-50%);
  font-variant-numeric:tabular-nums }}
.plot {{ height:var(--ph,200px) }}
.chart {{ width:100%; height:100%; display:block }}
.xaxis {{ display:flex; justify-content:space-between; padding-left:42px;
  font-size:11px; color:var(--muted); margin-top:4px }}
.grid {{ stroke:var(--line) }}
.tick {{ fill:var(--muted); font-size:10px }}
.legend {{ font-size:.82rem; color:var(--muted); margin-top:6px }}
.key i {{ display:inline-block; width:10px; height:10px; border-radius:2px; margin-right:5px }}
.key {{ margin-right:14px }}
.mark {{ stroke:var(--muted); stroke-dasharray:4 3 }}
.key i.ds {{ height:0; border-radius:0; border-top:2px dashed; vertical-align:3px }}
.key i.mk {{ width:0; height:11px; border-radius:0; border-left:2px dashed var(--muted);
  vertical-align:-1px }}
.muted {{ color:var(--muted) }}
.up {{ margin:0 0 12px; font-size:.86rem }}
.up a {{ color:var(--muted); text-decoration:none }}
.up a:hover {{ color:var(--fg) }}
.hot {{ color:#c2410c; font-weight:600 }}
.part {{ font-weight:400; color:var(--muted); font-size:.82em }}
.note {{ background:var(--card); border:1px solid var(--line); border-left:3px solid var(--muted);
  border-radius:0 8px 8px 0; padding:12px 16px; font-size:.88rem; color:var(--muted) }}
.warn {{ background:var(--warn); border:1px solid var(--line); border-radius:8px;
  padding:12px 16px; font-size:.88rem; margin:16px 0 }}
code {{ font-size:.85em }}
</style>
<main>
<p class="up"><a href="../">← KiCadRoutingTools</a></p>
<h1>KiCadRoutingTools — reach</h1>
<p class="sub">{_esc(slug)} · collected {_esc(meta.get('last_collected', 'never'))} ·
rebuilt daily from GitHub's API</p>

{err_html}

<div class="cards">
{''.join(f'<div class="card"><div class="l">{_esc(l)}</div><div class="n">{_esc(n)}</div><div class="h">{_esc(h)}</div></div>' for l, n, h in cards)}
</div>

<h2>Downloads by release</h2>
{dl_chart}
<p class="note"><strong>{dl_head}</strong> A release's counter is cumulative
and never stops rising, so before the archive began the only facts are each
release's lifetime total and its dates. Each total is spread evenly over the
release's <em>reign</em>: from when it became the newest release until its
successor took over. For the PCM zip both dates are when KiCad's catalogue
began serving a version, read from the merge history of
<code>kicad/addons/metadata</code>, because PCM serves only its newest listed
version and skips every release in between. The measured days show why this
is the right window: when a successor arrives, the old release drops to a
trickle within a day. So a level of interest that stays flat draws a flat
line, however many releases have been published.{dl_split} <strong>Two
known biases remain in the estimate:</strong> the trickle a superseded release
keeps gathering is counted inside its own reign, so the earliest plateaus read
slightly high, and even spread flattens the burst right after each release.
{dl_measured} Exact per-release totals are in the table below; the series
are never added together.</p>

<h2>Daily views and clones</h2>
{chart}
<p class="note"><strong>Uniques are not additive.</strong> GitHub reports a
unique count per day, and the same person on two days counts twice in any sum
of them — so the card above says “unique-days”, not “people”. Only the daily
figures are true uniques.</p>

<h2>Weekly activity</h2>
<div class="wrap"><table>
<tr><th>ISO week</th><th class="num">clones</th><th class="num">unique-days</th>
<th class="num">vs prev</th><th class="num">views</th></tr>
{wk_html or '<tr><td colspan="5" class="muted">no data yet</td></tr>'}
</table></div>
<p class="note"><strong>The newest week is almost always partial, and a
partial week always looks like a collapse</strong> — so the day count is shown
whenever it is under seven, and the week-over-week column is withheld rather
than computed against a stub. Lifetime since collection began
({banked} day{'' if banked == 1 else 's'} banked):
<strong>{lifetime_c:,}</strong> clones, <strong>{lifetime_v:,}</strong> views.
These totals only ever grow from here — the days before the first snapshot are
gone from GitHub and cannot be recovered.</p>

<h2>Manual vs automated clones</h2>
<div class="wrap"><table>
<tr><th>day (● = release)</th><th class="num">clones</th><th class="num">unique</th>
<th class="num">per unique</th><th class="num">views</th></tr>
{char_html or '<tr><td colspan="5" class="muted">no data yet</td></tr>'}
</table></div>
<p class="note"><strong>There is no way to count human clones, only to
estimate them.</strong> GitHub reports a count and a unique count and nothing
else — no user agent, no IP, no actor — so any figure here claiming to be
“people” would be invented. <strong>Unique cloners is the closest proxy</strong>,
because a person clones once or twice while an automated fetcher clones
repeatedly from few addresses; the ratio is therefore an automation index, not
a headcount. Ordinary days sit near 1.4–1.9.
{f'The peak in this window is <strong>{peak["ratio"]:.2f}</strong> on {_esc(peak["date"])}'
 + (' — a release day' if peak['release'] else '')
 + f', at {peak["count"]:,} clones against only {peak["views"]:,} views: machines, not readers.'
 if peak else ''}
This project's own CI is <em>not</em> the explanation and is not subtracted:
a release run is seven jobs, so about seven checkouts against a spike of
several hundred. The excess is other people's automation — downstream CI,
mirrors, release trackers — which no API available here can identify, so it is
disclosed rather than adjusted by a correction that would explain about 1% of
it.</p>

<h2>Downloads per release</h2>
<div class="wrap"><table>
<tr><th>release (PCM = listed)</th><th>published</th><th class="num">zip</th>
<th class="num">binaries (L/W/M)</th><th class="num">total</th></tr>
{''.join(rel_html)}
</table></div>
<p class="note"><strong>PCM and binaries are different audiences and are never
summed.</strong> KiCad's Plugin and Content Manager fetches the zip from
whichever release it currently points at, so PCM installs pile up on that one
release and a newer release showing few zip downloads means PCM has not been
pointed at it — not that interest collapsed. The <code>grid_router-*</code>
binaries are fetched by <code>build_router.py</code>, so they count
from-source installs, <em>including this project's own CI</em>: every Modal
image build downloads the Linux binary, which makes Linux an upper bound
rather than a user count.</p>

<h2>New downloads between snapshots</h2>
<div class="wrap"><table>
<tr><th>snapshot</th><th class="num">new PCM</th><th class="num">new direct zips</th>
<th class="num">new binaries</th></tr>
{delta_html or '<tr><td colspan="4" class="muted">needs two snapshots</td></tr>'}
</table></div>

<h2>Platform mix</h2>
{_bars(sorted(plat_tot.items(), key=lambda kv: -kv[1]))}

<h2>Where visitors come from</h2>
{_bars(ref_rows)}

<h2>What this cannot tell you</h2>
<p class="note">A download is not a run. Nothing here distinguishes one person
routing daily from a hundred who installed once and never opened it again, and
nothing here reports a crash, a failed route, or which KiCad or Python version
anyone is on. This project collects no telemetry, so those questions stay
unanswered by design. What these numbers <em>are</em> good for is reach,
platform mix, release adoption and trend — and for noticing when a week goes
unexpectedly quiet.</p>
</main>
"""
    out = os.path.join(SITE, 'metrics', 'index.html')
    os.makedirs(os.path.dirname(out), exist_ok=True)
    with open(out, 'w') as f:
        f.write(html)
    print(f'  wrote {_shown(out)} ({len(html):,} bytes)')
    land = write_landing(slug)
    print(f'  wrote {_shown(land)}')
    return out


def _shown(path):
    """`path` relative to the repo for the log line; as given when relpath
    cannot reach it (another Windows drive raises ValueError)."""
    try:
        return os.path.relpath(path, ROOT)
    except ValueError:
        return path


def write_landing(slug):
    """The site root: what this project is, and where the sub-pages are.

    Deliberately thin. It exists so that /metrics is a SUBPAGE rather than the
    whole site, leaving the root free for whatever comes next -- docs, a
    gallery, a demo -- without having to move a published URL again.
    """
    try:
        with open(os.path.join(ROOT, 'VERSION')) as f:
            version = f.read().strip()
    except Exception:
        version = ''
    repo = f'https://github.com/{slug}'
    html = f"""<!doctype html>
<meta charset="utf-8">
<meta name="viewport" content="width=device-width,initial-scale=1">
<title>KiCadRoutingTools</title>
<style>
:root {{ color-scheme: light dark;
  --bg:#fbfbfa; --fg:#1a1a18; --muted:#6b6b66; --line:#e2e2dd; --card:#fff; --accent:#2563eb; }}
@media (prefers-color-scheme: dark) {{ :root {{
  --bg:#16171a; --fg:#e8e8e4; --muted:#9a9a94; --line:#2c2e33; --card:#1d1e22; --accent:#60a5fa; }} }}
* {{ box-sizing:border-box }}
body {{ margin:0; padding:48px 16px 72px; background:var(--bg); color:var(--fg);
  font:16px/1.6 -apple-system,BlinkMacSystemFont,"Segoe UI",Roboto,sans-serif; }}
main {{ max-width:720px; margin:0 auto }}
h1 {{ font-size:2rem; margin:0 0 6px; letter-spacing:-.02em }}
.tag {{ color:var(--muted); margin:0 0 8px; font-size:1.05rem }}
.ver {{ color:var(--muted); font-size:.85rem; margin:0 0 32px }}
.links {{ display:grid; grid-template-columns:repeat(auto-fit,minmax(220px,1fr)); gap:12px; margin:0 0 32px }}
a.card {{ display:block; background:var(--card); border:1px solid var(--line);
  border-radius:10px; padding:16px 18px; text-decoration:none; color:inherit }}
a.card:hover {{ border-color:var(--accent) }}
a.card .t {{ font-weight:600; color:var(--accent) }}
a.card .d {{ font-size:.86rem; color:var(--muted); margin-top:3px }}
p {{ margin:0 0 16px }}
code {{ background:var(--card); border:1px solid var(--line); border-radius:4px;
  padding:1px 5px; font-size:.86em }}
.foot {{ color:var(--muted); font-size:.84rem; border-top:1px solid var(--line);
  padding-top:16px; margin-top:32px }}
</style>
<main>
<h1>KiCadRoutingTools</h1>
<p class="tag">An autorouter and placement toolkit for KiCad — a Rust grid router
with a Python engine, usable as CLI scripts or as a KiCad plugin.</p>
<p class="ver">{('Latest release v' + _esc(version)) if version else ''}</p>

<div class="links">
  <a class="card" href="{repo}"><div class="t">Source &amp; docs →</div>
    <div class="d">The repository, issues, and the tool reference</div></a>
  <a class="card" href="{repo}/releases"><div class="t">Releases →</div>
    <div class="d">Plugin package and prebuilt router binaries</div></a>
  <a class="card" href="metrics/"><div class="t">Reach metrics →</div>
    <div class="d">Installs, downloads and traffic, updated daily</div></a>
  <a class="card" href="solvers/"><div class="t">How the whole route plans →</div>
    <div class="d">Interactive explainer of CP-SAT and HiGHS in the bus router</div></a>
  <a class="card" href="solvers/dual/"><div class="t">Shadow prices as forces →</div>
    <div class="d">The geometry LP's dual as statics, geometry and a circuit</div></a>
</div>

<p>This is the project's page on the web: links to the source and the
releases, the project's reach, and interactive explainers of how the router
works.</p>

<div class="foot">This site is built from the repository and republished daily,
and whenever a release is published, by a GitHub Actions workflow.</div>
</main>
"""
    os.makedirs(SITE, exist_ok=True)
    out = os.path.join(SITE, 'index.html')
    with open(out, 'w') as f:
        f.write(html)
    # hand-written pages kept under docs/ (each a folder with an index.html)
    for page in ('solvers',):
        src = os.path.join(ROOT, 'docs', page)
        if os.path.isdir(src):
            shutil.copytree(src, os.path.join(SITE, page), dirs_exist_ok=True)
    return out


def main():
    ap = argparse.ArgumentParser(
        description='Snapshot GitHub reach data into the archive (default '
                    'metrics/data) and render the site into docs/site.')
    ap.add_argument('--repo', default='', help='owner/name (default: git remote)')
    ap.add_argument('--token', default='', help='API token (default: env/gh CLI)')
    ap.add_argument('--only', default='collect,render',
                    help='collect,render (default: both)')
    ap.add_argument('--data-dir', default='',
                    help='snapshot archive directory (default: metrics/data, '
                         'or $KRT_METRICS_DATA). CI points this at a checkout '
                         'of the orphan metrics-data branch.')
    ap.add_argument('--site-dir', default='',
                    help='rendered site directory (default: docs/site, or '
                         '$KRT_METRICS_SITE)')
    args = ap.parse_args()

    # Module-level so every collect/render helper sees the override without
    # threading a path through all of them; the archive test already rebinds
    # these two the same way.
    global DATA, SITE
    if args.data_dir:
        DATA = os.path.abspath(args.data_dir)
    if args.site_dir:
        SITE = os.path.abspath(args.site_dir)
    os.makedirs(DATA, exist_ok=True)
    print(f'archive: {DATA}')
    print(f'site:    {SITE}')

    slug = _repo_slug(args.repo)
    stages = [s.strip() for s in args.only.split(',') if s.strip()]
    print(f'repo: {slug}')
    errors, collected = {}, ['(not run)']
    if 'collect' in stages:
        print('=== COLLECT ===')
        errors, collected = collect(slug, args.token)
    if 'render' in stages:
        print('=== RENDER ===')
        render(slug)
    # A failed endpoint is reported, recorded and RENDERED -- but must not fail
    # the run, because the publish step is downstream and a non-zero exit here
    # kills the very page that carries the disclosure. The first CI run died
    # exactly that way: four traffic 403s aborted the job after the page had
    # been written correctly, so nobody could read what it said. Only a TOTAL
    # loss -- nothing collected at all -- is worth a non-zero exit.
    return 1 if errors and not collected else 0


if __name__ == '__main__':
    sys.exit(main())
