#!/usr/bin/env python3
"""Snapshot GitHub's reach data into a git-tracked archive, and render it.

WHY THIS EXISTS: GitHub's traffic API is a ROLLING 14-DAY WINDOW and is not
retroactive. Everything older is discarded by GitHub and cannot be recovered by
anyone. A weekly snapshot committed to the repo is the only way this project
ever has a history of its own reach. Release asset counts do not expire, but
they are CUMULATIVE totals with no per-period breakdown, so the only way to
learn "how many downloads last week" is to diff two snapshots -- which again
requires keeping them.

Weekly is sufficient, and that is a property of the API rather than a guess:
each traffic call returns FOURTEEN daily buckets, so consecutive runs up to 14
days apart still observe every day. Merging is by date, keeping the max, so the
overlap between runs is idempotent and a partially-elapsed day is corrected by
the next run rather than frozen at its partial value.

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
import subprocess
import sys
import urllib.error
import urllib.request
from datetime import datetime, timezone

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
DATA = os.path.join(ROOT, 'metrics', 'data')
SITE = os.path.join(ROOT, 'docs', 'site')

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
        _save(fname, store)
        collected.append(ep)
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
        _save('releases.json', store)
        collected.append('releases')
        print(f'  releases: {len(snap)} release(s) snapshotted')

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


def _line_chart(series, width=880, height=200, pad=32):
    """Inline SVG, no dependencies -- the page must render from a file:// URL."""
    days = sorted(set().union(*[set(s['points']) for s in series if s['points']])
                  or {''})
    days = [d for d in days if d]
    if not days:
        return '<p class="muted">no data yet</p>'
    top = max([max(s['points'].values() or [0]) for s in series] + [1])
    w, h = width - 2 * pad, height - 2 * pad

    def xy(i, v):
        x = pad + (w * i / max(1, len(days) - 1))
        y = pad + h - (h * v / top)
        return f'{x:.1f},{y:.1f}'

    out = [f'<svg viewBox="0 0 {width} {height}" class="chart" '
           f'preserveAspectRatio="none" role="img">']
    for frac in (0, 0.5, 1):
        y = pad + h - h * frac
        out.append(f'<line x1="{pad}" y1="{y:.1f}" x2="{pad + w}" y2="{y:.1f}" '
                   f'class="grid"/>')
        out.append(f'<text x="4" y="{y + 4:.1f}" class="tick">{int(top * frac)}</text>')
    for s in series:
        pts = ' '.join(xy(i, s['points'].get(d, 0)) for i, d in enumerate(days))
        out.append(f'<polyline points="{pts}" fill="none" stroke="{s["color"]}" '
                   f'stroke-width="2" stroke-linejoin="round"/>')
    out.append(f'<text x="{pad}" y="{height - 8}" class="tick">{days[0]}</text>')
    out.append(f'<text x="{pad + w}" y="{height - 8}" class="tick" '
               f'text-anchor="end">{days[-1]}</text>')
    out.append('</svg>')
    legend = ' '.join(f'<span class="key"><i style="background:{s["color"]}"></i>'
                      f'{_esc(s["label"])}</span>' for s in series)
    return ''.join(out) + f'<div class="legend">{legend}</div>'


def spread_downloads(rows, today=None):
    """Per-release lifetime totals -> an estimated downloads-per-day timeline.

    WHY NOT BARS. A release's counter is cumulative and never stops rising, and
    PCM piles every install onto whichever release it points at -- so a bar
    chart shows three towers and thirty-six stubs. That is true but unreadable,
    and it says nothing about WHEN any of it happened.

    Each release's total is spread evenly across its lifetime (publish date to
    today) and the per-day contributions are summed, which turns the towers
    into overlapping plateaus: a release that gathered 4,000 installs over a
    month reads as ~130/day for that month, directly comparable to one that
    gathered 200 over the same span.

    TWO BIASES, BOTH DISCLOSED ON THE PAGE RATHER THAN HIDDEN:

    1. Even spread is wrong in a known direction -- downloads arrive fastest
       just after a release and taper -- so the start of each plateau is
       understated and the tail overstated.
    2. The TOTAL slopes upward as an artifact: every release ever published
       keeps contributing to every later day, so the sum grows with the size of
       the catalogue even if interest is flat. The shape of the plateaus is
       meaningful; the trend of their sum is not.

    Both are TEMPORARY. Once two snapshots exist, differencing them gives the
    real per-period rate with no assumption at all, which is what
    `_weekly_deltas` already does for the table.
    """
    from datetime import date as _date, timedelta
    if today is None:
        today = _date(*map(int, _today().split('-')))
    out = {}
    for r in rows:
        pub = (r.get('published') or '')[:10]
        if not pub:
            continue
        try:
            start = _date(*map(int, pub.split('-')))
        except Exception:
            continue
        days = max(1, (today - start).days + 1)
        pcm = r['pcm'] / days
        binr = sum(r['binaries'].values()) / days
        for i in range(days):
            key = (start + timedelta(days=i)).isoformat()
            cell = out.setdefault(key, {'pcm': 0.0, 'bin': 0.0})
            cell['pcm'] += pcm
            cell['bin'] += binr
    return out


def _grouped_bars(rows, series, width=880, height=220, pad=34):
    """Two bars per category, inline SVG. `rows` = [(label, {key: value})].

    Linear, not log, and deliberately: PCM installs concentrate on whichever
    release PCM points at, so one bar towering over the rest IS the finding.
    A log axis would flatten it into a tidy picture that hides the shape of how
    this project is actually installed.
    """
    if not rows:
        return '<p class="muted">no data yet</p>'
    top = max([v for _, d in rows for v in d.values()] + [1])
    w, h = width - 2 * pad, height - 2 * pad
    slot = w / max(1, len(rows))
    bw = min(14.0, max(2.0, slot / (len(series) + 1)))
    out = [f'<svg viewBox="0 0 {width} {height}" class="chart chart-tall" '
           f'preserveAspectRatio="none" role="img">']
    for frac in (0, 0.5, 1):
        y = pad + h - h * frac
        out.append(f'<line x1="{pad}" y1="{y:.1f}" x2="{pad + w}" y2="{y:.1f}" class="grid"/>')
        out.append(f'<text x="2" y="{y + 4:.1f}" class="tick">{int(top * frac):,}</text>')
    for i, (label, vals) in enumerate(rows):
        base = pad + i * slot + (slot - bw * len(series)) / 2
        for j, (key, colour, sname) in enumerate(series):
            v = vals.get(key, 0)
            bh = h * v / top
            x = base + j * bw
            out.append(
                f'<rect x="{x:.1f}" y="{pad + h - bh:.1f}" width="{bw - 1:.1f}" '
                f'height="{max(bh, 0.6):.1f}" fill="{colour}">'
                f'<title>{_esc(label)} — {_esc(sname)}: {v:,}</title></rect>')
    out.append(f'<text x="{pad}" y="{height - 8}" class="tick">{_esc(rows[0][0])}</text>')
    out.append(f'<text x="{pad + w}" y="{height - 8}" class="tick" '
               f'text-anchor="end">{_esc(rows[-1][0])}</text>')
    out.append('</svg>')
    legend = ' '.join(f'<span class="key"><i style="background:{c}"></i>{_esc(n)}</span>'
                      for _k, c, n in series)
    return ''.join(out) + f'<div class="legend">{legend}</div>'


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


def _weekly_deltas(releases):
    """Per-snapshot NEW downloads: cumulative counters differenced in time."""
    stamps = sorted(releases)
    out = []
    for prev, cur in zip(stamps, stamps[1:]):
        a, b = releases[prev], releases[cur]

        def total(snap, pred):
            return sum(v for rel in snap.values()
                       for k, v in rel.get('assets', {}).items() if pred(k))
        pcm = total(b, lambda k: bool(_PCM_RE.match(k))) - \
            total(a, lambda k: bool(_PCM_RE.match(k)))
        names = {n for n, _ in PLATFORMS}
        binr = total(b, lambda k: k in names) - total(a, lambda k: k in names)
        out.append({'date': cur, 'pcm': max(0, pcm), 'bin': max(0, binr)})
    return out


def render(slug):
    traffic = _load('traffic_daily.json', {})
    releases = _load('releases.json', {})
    referrers = _load('referrers.json', {})
    meta = _load('meta.json', {})

    rows, plat_tot, pcm_tot = _release_rollup(releases)
    deltas = _weekly_deltas(releases)
    views, clones = traffic.get('views', {}), traffic.get('clones', {})
    last14 = sorted(views)[-14:]

    def _sum(d, days, key):
        return sum(d.get(x, {}).get(key, 0) for x in days)

    pcm_head = max(pcm_tot.items(), key=lambda kv: kv[1]) if pcm_tot else ('-', 0)
    cards = [
        ('PCM installs', f'{sum(pcm_tot.values()):,}',
         f'all releases; {pcm_head[1]:,} on {pcm_head[0]}'),
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
            f'<tr><th>{_esc(r["tag"])}</th><td>{_esc(r["published"])}</td>'
            f'<td class="num">{r["pcm"]:,}</td><td class="num">{b}</td>'
            f'<td class="num">{r["total"]:,}</td></tr>')

    delta_html = ''.join(
        f'<tr><th>{_esc(d["date"])}</th><td class="num">{d["pcm"]:,}</td>'
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

    spread = spread_downloads(rows)
    dl_chart = _line_chart([
        {'label': 'PCM zip installs/day', 'color': '#3b82f6',
         'points': {d: v['pcm'] for d, v in spread.items()}},
        {'label': 'router binaries/day', 'color': '#f97316',
         'points': {d: v['bin'] for d, v in spread.items()}},
    ], height=220)

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
.chart {{ width:100%; height:200px; display:block }}
.chart-tall {{ height:220px }}
.grid {{ stroke:var(--line) }}
.tick {{ fill:var(--muted); font-size:10px }}
.legend {{ font-size:.82rem; color:var(--muted); margin-top:6px }}
.key i {{ display:inline-block; width:10px; height:10px; border-radius:2px; margin-right:5px }}
.key {{ margin-right:14px }}
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
rebuilt weekly from GitHub's API</p>

{err_html}

<div class="cards">
{''.join(f'<div class="card"><div class="l">{_esc(l)}</div><div class="n">{_esc(n)}</div><div class="h">{_esc(h)}</div></div>' for l, n, h in cards)}
</div>

<h2>Downloads by release</h2>
{dl_chart}
<p class="note"><strong>Estimated rate, not a measurement — yet.</strong> A
release's counter is cumulative and never stops rising, and PCM piles every
install onto whichever release it points at, so the raw numbers are three
towers and thirty-six stubs. Here each release's lifetime total is spread
evenly across the days since it was published and the contributions are summed,
which makes a release that gathered 4,000 installs over a month read as ~130/day
rather than one spike. <strong>Even spread is an assumption, and it is wrong in
a known direction:</strong> downloads arrive fastest just after a release and
taper, so the start of each plateau is understated and the tail overstated. It
is temporary — once two weekly snapshots exist, differencing them gives the
real per-period rate with no assumption at all. <strong>The upward slope is
partly an artifact of the method</strong> for the same reason: every release
ever published keeps contributing to every later day, so the total rises as the
catalogue grows even if interest is flat. Read the SHAPE of the plateaus, not
the trend. Exact per-release totals are in the table below; the two series are
still never added together.</p>

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
<tr><th>release</th><th>published</th><th class="num">PCM zip</th>
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
<tr><th>snapshot</th><th class="num">new PCM</th><th class="num">new binaries</th></tr>
{delta_html or '<tr><td colspan="3" class="muted">needs two snapshots</td></tr>'}
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
    print(f'  wrote {os.path.relpath(out, ROOT)} ({len(html):,} bytes)')
    land = write_landing(slug)
    print(f'  wrote {os.path.relpath(land, ROOT)}')
    return out


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
    <div class="d">Installs, downloads and traffic, updated weekly</div></a>
</div>

<p>Install through KiCad's <strong>Plugin and Content Manager</strong>, or clone
the repository and run <code>python3 build_router.py</code> to fetch the
prebuilt router for your platform.</p>

<div class="foot">This site is built from the repository and republished weekly
by a GitHub Actions workflow.</div>
</main>
"""
    os.makedirs(SITE, exist_ok=True)
    out = os.path.join(SITE, 'index.html')
    with open(out, 'w') as f:
        f.write(html)
    return out


def main():
    ap = argparse.ArgumentParser(
        description='Snapshot GitHub reach data into metrics/data and render '
                    'docs/metrics/index.html.')
    ap.add_argument('--repo', default='', help='owner/name (default: git remote)')
    ap.add_argument('--token', default='', help='API token (default: env/gh CLI)')
    ap.add_argument('--only', default='collect,render',
                    help='collect,render (default: both)')
    args = ap.parse_args()

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
