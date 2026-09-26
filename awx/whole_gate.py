"""whole_gate.py PLAN.json AUDIT.txt [--hot OUT.json] -- exit 0 when the plan is COMPLETE and passes the audit: every corridor member
laid (a plan with a lane missing, or one its maker marked failed, fails here -- audited lane by lane, a missing lane
has nothing to fail on; so does one the snap could only lay folding against its stub), no dive, static, shape or swim
failure, no planned length outside its band, and no pitch failure. (A lane's terminal join is graded as the router
lays it -- plan_audit._router_terminals -- so two teeth closer than the plan's own bar are measured, not waived.)

The audit is read by its own summary lines, and an audit that lacks one did not run to its end (a crash leaves a
traceback and no counts): that fails too, as does a pitch count the per-pair lines do not add up to.

--hot OUT.json writes where the audit found the plan short -- every dive, pitch, static and shape finding's position,
{"hot": [[x, y, kind], ...]} -- the history the solve prices (whole_solve, HIST)."""
import json
import re
import sys

geo = json.load(open(sys.argv[1]))
aud = open(sys.argv[2]).read().splitlines()
if '--hot' in sys.argv:
    NUM_ = r'(-?\d+(?:\.\d+)?)'
    hot = []
    for line in aud:
        k = line.split(' ', 1)[0]
        m = re.search(rf'\(\s*{NUM_},\s*{NUM_}\)', line) if k in ('DIVE', 'PITCH', 'STATIC', 'SHAPE') else None
        if m:
            hot.append([float(m.group(1)), float(m.group(2)), k])
    json.dump({'hot': hot}, open(sys.argv[sys.argv.index('--hot') + 1], 'w'))
lanes = set(geo['lanes'])

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
plan = []
for line in aud:
    m = re.match(rf'^PITCH (\S+)\s+(\S+)\s+\S min {NUM}\s+over [\d.]+ mm\s+at \({NUM},\s*{NUM}\)', line)
    if m:
        plan.append(f'{m.group(1)}/{m.group(2)} {float(m.group(3)):.3f}')
unread = int(found['PITCH']) - len(plan)
folded = geo.get('folded', [])
bad = geo.get('failed') or missing or unread or folded
print(('INCOMPLETE: ' + (f'{len(missing)} lane(s) missing: {", ".join(missing)}; ' if missing else '')
       + ('its maker marked it failed; ' if geo.get('failed') else '') if (geo.get('failed') or missing) else '')
      + (f'UNREAD: {unread} pitch line(s) the gate could not parse; ' if unread else '')
      + (f'FOLDED: {", ".join(folded)} laid with no approach within 90 degrees of a stub; ' if folded else '')
      + f'audit: dive {fails["DIVE"]}, static {fails["STATIC"]}, shape {fails["SHAPE"]}, swim {fails["SWIM"]}, '
        f'band {fails["BAND"]:.1f} mm, pitch {len(plan)} in the plan' + (f': {plan}' if plan else ''))
sys.exit(0 if not bad and not plan and not any(fails.values()) else 1)
