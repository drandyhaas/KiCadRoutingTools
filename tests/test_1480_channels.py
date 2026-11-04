"""#1480: replicate copies channels drawn INSIDE one sheet, named by parts.

`replicate` paired sibling sheet instances by their shared symbol uuid, so
channels drawn as copies inside one sheet could not be paired. `channels`
names them by their parts, matched index for index, and every check of the
sheet form runs on them; `suggest_channels` proposes them.

The fixture board draws three line-driver channels (a chip and four
charge-pump caps each) in one sheet, laid out by hand as translated copies.
"""

import os
import subprocess
import sys
import tempfile

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
for _p in ('py_placer', 'py_router', 'py_tools', 'tests'):
    sys.path.insert(0, os.path.join(ROOT, _p))

from kicad_parser import parse_kicad_pcb                      # noqa: E402
from placement import floorplan as fp                         # noqa: E402
from placement import replicate as rp                         # noqa: E402
from placement.pose_ops import PoseRefusal                    # noqa: E402

BOARD = os.path.join(ROOT, 'kicad_files', 'kit-dev-coldfire-xilinx_5213.kicad_pcb')
CHIPS = ('U202', 'U203', 'U204')
FAILS = []


def ok(name, cond, detail=''):
    print(f"  {'PASS' if cond else 'FAIL'}  {name}"
          + (f"  ({detail})" if detail and not cond else ''))
    if not cond:
        FAILS.append(name)


def refused(fn):
    try:
        fn()
    except PoseRefusal as e:
        return str(e)
    return None


def main():
    pcb = parse_kicad_pcb(BOARD)
    declined = []
    sugg = rp.suggest_channels(pcb, declined=declined)
    ok('the three channels are proposed', len(sugg) == 1 and sugg[0]['chips'] == list(CHIPS),
       [s['chips'] for s in sugg])
    channels = sugg[0]['channels'] if sugg else []
    ok('each with its four caps', [len(c) for c in channels] == [5, 5, 5], channels)
    ok('a chip group with nothing matched is declined, not proposed',
       all('refused' in d for d in declined) and declined)
    rows, report = rp.plan(pcb, None, (), channels=channels)
    ok('two copies of the source', [t['sheet']['path'] for t in report['to']]
       == ['/channel:U203', '/channel:U204'], report['to'])
    ok('each anchored on its chip', all(t['offset_basis']['source'] == 'U202'
                                        for t in report['to']))
    ok('the hand layout is a copy within a quarter millimetre',
       all(abs(r['x'] - pcb.footprints[r['ref']].x) < 0.3
           and abs(r['y'] - pcb.footprints[r['ref']].y) < 0.3 for r in rows))
    ok('unequal channels are refused',
       'same length' in (refused(lambda: rp.plan(
           pcb, None, (), channels=[channels[0], channels[1][:-1]])) or ''))
    ok('a part in two channels is refused',
       'one channel only' in (refused(lambda: rp.plan(
           pcb, None, (), channels=[channels[0], channels[0]])) or ''))
    # declared in the intent: graded by `replicate`, applied by place_pose
    it = fp.intent_from_dict({'schema': 1, 'kind': fp.KIND,
                              'min_reader': fp.CHANNELS_READER,
                              'replicate': [{'channels': channels}]})
    ok('the loader names a channels row by its first refs',
       it.replicate[0]['from'] == 'channel:U202'
       and it.replicate[0]['to'] == ['channel:U203', 'channel:U204'], it.replicate[0])
    meas = rp.measure_declared(pcb, it.replicate[0])
    ok('the grade fits one translation per copy',
       [round(m['fit_offset'][0], 2) for m in meas] == [-27.94, -60.96], meas)
    with tempfile.TemporaryDirectory() as td:
        out = os.path.join(td, 'o.kicad_pcb')
        r = subprocess.run([sys.executable, '-X', 'utf8',
                            os.path.join(ROOT, 'py_placer', 'place_pose.py'), BOARD, out,
                            'replicate'] + sum((['--channel', ','.join(c)] for c in channels), []),
                           capture_output=True, text=True, cwd=ROOT)
        ok('place_pose replicate --channel writes the copy', r.returncode == 0
           and os.path.exists(out), r.stdout[-300:])
        if os.path.exists(out):
            after = rp.measure_declared(parse_kicad_pcb(out), it.replicate[0])
            ok('after it each copy is exact', all(m['max_residual_mm'] == 0.0 for m in after),
               [m['max_residual_mm'] for m in after])
    try:
        fp.intent_from_dict({'schema': 1, 'kind': fp.KIND, 'replicate': [
            {'channels': channels, 'from': 'X', 'to': ['Y']}]})
        ok('channels with from/to refused', False)
    except fp.IntentError as e:
        ok('channels with from/to refused', 'declare one' in str(e), str(e))
    print('ALL PASS' if not FAILS else f'FAILED: {FAILS}')
    return 1 if FAILS else 0


if __name__ == '__main__':
    sys.exit(main())
