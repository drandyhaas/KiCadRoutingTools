"""whole_gate.py PLAN.json AUDIT.txt [--hot OUT.json] -- exit 0 when the plan is COMPLETE and passes the audit: every corridor member
laid (a plan with a lane missing, or one its maker marked failed, fails here -- audited lane by lane, a missing lane
has nothing to fail on; so does one the snap could only lay folding against its stub), no dive, static, shape or swim
failure, no planned length outside its band, every lane's band joining its tooth to its berth (the router is confined
to it: a band BROKEN, or not reaching its tooth or its berth, fails), and no pitch failure. (A lane's terminal join is graded as the router
lays it -- plan_audit._router_terminals -- so two teeth closer than the plan's own bar are measured, not waived.)

The audit is read by its own summary lines, and an audit that lacks one did not run to its end (a crash leaves a
traceback and no counts): that fails too, as does a pitch count the per-pair lines do not add up to.

--hot OUT.json writes where the audit found the plan short -- every dive, pitch, static and shape finding's position,
and the lanes it names, and every lane a snap could not lay where its search got stuck (whole_snap's conflicts, SNAP),
{"hot": [[x, y, kind, [lanes]], ...]} -- the history the solve prices (whole_solve, HIST) and the ends the fanout is
asked to avoid (whole_feedback)."""
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
        m = re.search(rf'\(\s*{NUM_},\s*{NUM_}\)', line) if k in ('DIVE', 'PITCH', 'STATIC', 'SHAPE', 'LINT') else None
        if m:
            w = line.split()
            # the lanes it names: PITCH two; STATIC its lane and a run net's copper it runs against; SHAPE its lane;
            # DIVE its lane and the lane it stands too near
            ln = w[1:3] if k == 'PITCH' else w[2:3] if k == 'LINT' else w[1:2]      # (LINT: its kind, then the lane)
            if k == 'STATIC' and 'copper' in w:
                ln = ln + w[w.index('copper') + 1:w.index('copper') + 2]
            if k == 'DIVE':
                ln = ln + re.findall(r'\(([^\s,()]+)(?: (?:[FB]|In\d+\.Cu))?\)', line)   # '(LANE F)' a line, '(LANE)' a via
            hot.append([float(m.group(1)), float(m.group(2)), k, ln])
    for c_ in geo.get('conflicts', ()):
        if c_.get('frame') == 'snap':
            hot.append([float(c_['xy'][0]), float(c_['xy'][1]), 'SNAP', list(c_['lanes'])])
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
# a lane whose band does not join its tooth to its berth (BROKEN, TOOTH-OUT, BERTH-OUT) fails, whatever length of it
# lies outside: the router is confined to the band
broken = sorted(m.group(1) for line in aud
                for m in [re.match(r'^BAND (\S+)\s+\S+\s+plan\s+[\d.]+ mm\s+outside\s+[\d.]+ mm\s+(\S+)', line)]
                if m and m.group(2) != 'connected')
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
    m = re.match(rf'^PITCH (\S+)\s+(\S+)\s+(?:[FB]|In\d+\.Cu) min {NUM}\s+over [\d.]+ mm\s+at \({NUM},\s*{NUM}\)', line)
    if m:
        plan.append(f'{m.group(1)}/{m.group(2)} {float(m.group(3)):.3f}')
unread = int(found['PITCH']) - len(plan)
folded = geo.get('folded', [])
bad = geo.get('failed') or missing or unread or folded or broken
print(('INCOMPLETE: ' + (f'{len(missing)} lane(s) missing: {", ".join(missing)}; ' if missing else '')
       + ('its maker marked it failed; ' if geo.get('failed') else '') if (geo.get('failed') or missing) else '')
      + (f'UNREAD: {unread} pitch line(s) the gate could not parse; ' if unread else '')
      + (f'FOLDED: {", ".join(folded)} laid with no approach within 90 degrees of a stub; ' if folded else '')
      + f'audit: dive {fails["DIVE"]}, static {fails["STATIC"]}, shape {fails["SHAPE"]}, swim {fails["SWIM"]}, '
        f'band {fails["BAND"]:.3f} mm' + (f', band broken {len(broken)}: {broken}' if broken else '')
      + f', pitch {len(plan)} in the plan' + (f': {plan}' if plan else ''))
sys.exit(0 if not bad and not plan and not any(fails.values()) else 1)
