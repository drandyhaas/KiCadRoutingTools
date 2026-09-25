"""whole_gate.py PLAN.json AUDIT.txt -- exit 0 when the plan is COMPLETE and passes the audit: every corridor member
laid (a plan with a lane missing, or one its maker marked failed, fails here -- audited lane by lane, a missing lane
has nothing to fail on; so does one the snap could only lay folding against its stub), no dive, static, shape or swim
failure, no planned length outside its band, and no pitch failure but at two lanes' fixed ends that are themselves
closer than the bar (the fanout's copper, not the plan's -- reported by name, never waived silently).

The audit is read by its own summary lines, and an audit that lacks one did not run to its end (a crash leaves a
traceback and no counts): that fails too, as does a pitch count the per-pair lines do not add up to."""
import json
import math
import re
import sys

geo = json.load(open(sys.argv[1]))
aud = open(sys.argv[2]).read().splitlines()
lanes = set(geo['lanes'])
g = geo['rules']['grid']

SUMMARY = {'PITCH': r'^PITCH (\d+) pair\(s\) short', 'DIVE': r'^DIVE \d+ planned via sites.*failing: (\{.*\})',
           'STATIC': r'^STATIC (\d+) lane/object', 'SHAPE': r'^SHAPE (\d+) place', 'SWIM': r'^SWIM (\d+) place',
           'BAND': r'^BAND total planned length outside its band: ([\d.]+) mm'}
found = {}
for line in aud:
    for k, pat in SUMMARY.items():
        m = re.match(pat, line)
        if m:
            found[k] = m.group(1)
members = {m.group(1) for line in aud for m in [re.match(r'^BAND (\S+)\s', line)] if m and m.group(1) != 'total'}
unplanned = {m.group(1) for line in aud for m in [re.match(r'^BAND (\S+)\s+no planned line', line)] if m}
lost = sorted(k for k in SUMMARY if k not in found) + (['the corridor members'] if not members else [])
if lost:
    print(f'AUDIT INCOMPLETE: no {", ".join(lost)} line(s) in {sys.argv[2]} -- the audit did not run to its end')
    sys.exit(1)
fails = {'DIVE': sum(int(v) for v in re.findall(r':\s*(\d+)', found['DIVE'])), 'STATIC': int(found['STATIC']),
         'SHAPE': int(found['SHAPE']), 'SWIM': int(found['SWIM']), 'BAND': float(found['BAND'])}
missing = sorted((members - lanes) | unplanned)

NUM = r'(-?[\d.]+)'
ends = {n: (v['xy'][0], v['xy'][-1]) for n, v in geo['lanes'].items()}
fixed, plan = [], []
for line in aud:
    m = re.match(rf'^PITCH (\S+)\s+(\S+)\s+\S min {NUM}\s+over [\d.]+ mm\s+at \({NUM},\s*{NUM}\)', line)
    if not m:
        continue
    a, b, d, x, y = m.group(1), m.group(2), float(m.group(3)), float(m.group(4)), float(m.group(5))
    near = lambda n: min(math.hypot(e[0] - x, e[1] - y) for e in ends[n])
    ea = min(ends[a], key=lambda e: math.hypot(e[0] - x, e[1] - y))
    eb = min(ends[b], key=lambda e: math.hypot(e[0] - x, e[1] - y))
    gap = math.hypot(ea[0] - eb[0], ea[1] - eb[1])
    # the plan comes no nearer than the two ends already stand (within half a grid step), where they stand
    if min(near(a), near(b)) < 4 * g and max(near(a), near(b)) < d + 4 * g and gap <= d + g / 2:
        fixed.append(f'{a}/{b} ends {gap:.3f} apart')
    else:
        plan.append(f'{a}/{b} {d:.3f}')
unread = int(found['PITCH']) - len(fixed) - len(plan)
folded = geo.get('folded', [])
bad = geo.get('failed') or missing or unread or folded
print(('INCOMPLETE: ' + (f'{len(missing)} lane(s) missing: {", ".join(missing)}; ' if missing else '')
       + ('its maker marked it failed; ' if geo.get('failed') else '') if (geo.get('failed') or missing) else '')
      + (f'UNREAD: {unread} pitch line(s) the gate could not parse; ' if unread else '')
      + (f'FOLDED: {", ".join(folded)} laid with no approach within 90 degrees of a stub; ' if folded else '')
      + f'audit: dive {fails["DIVE"]}, static {fails["STATIC"]}, shape {fails["SHAPE"]}, swim {fails["SWIM"]}, '
        f'band {fails["BAND"]:.1f} mm, pitch {len(plan)} in the plan'
      + (f' + {len(fixed)} at fixed ends closer than the bar (fanout): {fixed}' if fixed else ''))
sys.exit(0 if not bad and not plan and not any(fails.values()) else 1)
