#!/usr/bin/env python3
"""#2067: the stitch vias route_planes.py writes keep their net in KiCad.

A stitching via's only connection is its pour. The planes step writes the new
pour as an outline (no fill), and the board keeps the stored fill of every
zone the step does not replace -- here a GND zone on B.Cu, as KiCad saved it.
`pcbnew.LoadBoard` (and kicad-cli, which loads before it refills) gives a
via that is not `(free yes)` the net of the stored fill it touches: before the
fix 1240 of the 1265 VCC stitch vias loaded as GND, and kicad-cli graded 199
of them dangling GND vias. KiCad's own via tool marks a via placed on a zone
with no track or pad under it free, and the tool now does the same for every
stitch and fence via.

1. (no KiCad) the planes step on the in-repo board, which carries the stored
   GND fill: every via it adds is written `(free yes)` and the parser reads it
   free and VCC.
2. (no KiCad) the token's spelling follows the file format: a KiCad 6/7 file
   takes a bare `(free)` (its reader refuses `(free yes)`), and the parser
   reads both spellings.
3. (no KiCad) control: the vias a route.py step adds on the same board -- each
   on a track to its own pad -- are NOT written free.
4. (KiCad's python) `pcbnew.LoadBoard` reads every stitch via with the net the
   file writes. Exits 77 (SKIP) after 1-3 when KiCad's python is absent.

    python3 tests/test_2067_stitch_vias_free.py [--workdir DIR]
"""
import argparse
import json
import os
import re
import subprocess
import sys
import tempfile

TESTS = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.dirname(TESTS)
sys.path.insert(0, os.path.join(ROOT, 'py_router'))
sys.path.insert(0, TESTS)

from run_utils import check, evidence, tool                    # noqa: E402

SKIP_EXIT = 77
BOARD = os.path.join(ROOT, 'kicad_files', 'lvds_converter_dualclk_gnd.kicad_pcb')
VIA_RE = re.compile(r'\(via\b[^\n]*\n(?:\t\t[^\n]*\n)*?\t\)')

#: Runs in KiCad's python: {uuid: net name} for the vias whose uuids it is given.
PROBE = r'''
import json, sys
import pcbnew
want = set(json.load(open(sys.argv[2])))
b = pcbnew.LoadBoard(sys.argv[1])
print('RESULT ' + json.dumps({t.m_Uuid.AsString(): t.GetNetname() for t in b.GetTracks()
                              if isinstance(t, pcbnew.PCB_VIA) and t.m_Uuid.AsString() in want}))
'''


def _vias(path):
    with open(evidence(path), encoding='utf-8') as f:
        return VIA_RE.findall(f.read())


def _new_vias(before, after):
    """The via blocks of `after` whose position `before` does not have."""
    at = re.compile(r'\(at ([-\d.]+) ([-\d.]+)\)')
    old = {tuple(map(float, at.search(b).groups())) for b in _vias(before)}
    return [b for b in _vias(after) if tuple(map(float, at.search(b).groups())) not in old]


def _uuid(block):
    return re.search(r'\(uuid "([^"]+)"\)', block).group(1)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--workdir', help='scratch directory (default: a new temp dir)')
    args = ap.parse_args()
    d = args.workdir or tempfile.mkdtemp(prefix='t2067_')
    os.makedirs(d, exist_ok=True)
    py = [sys.executable, '-X', 'utf8', '-B']
    from kicad_parser import parse_kicad_pcb, extract_vias

    src = os.path.join(d, 'in.kicad_pcb')
    check(py + [tool('copy_board.py'), evidence(BOARD, 'the in-repo board'), src], accept=True)
    with open(evidence(src), encoding='utf-8') as f:
        text_in = f.read()
    # The precondition the defect needs: a stored fill of ANOTHER net.
    assert re.search(r'\(zone\s+\(net "/GND"\)\s+\(layer "B\.Cu"\)', text_in) \
        and text_in.count('(filled_polygon') == 1, \
        'the fixture no longer carries its one stored GND fill on B.Cu -- this test tests nothing'

    print('1. the planes step writes its stitch vias free')
    out = os.path.join(d, 'planes.kicad_pcb')
    r = check(py + [tool('route_planes.py'), src, out, '--nets', '/VCC', '/VCC',
                    '--plane-layers', 'F.Cu', 'B.Cu', '--stitch-vias',
                    '--stitch-pitch', '1.0'], accept=True)
    m = re.search(r'Via stitching total: (\d+) via\(s\) added', r.stdout)
    assert m and int(m.group(1)) > 1000, 'the stitch pass did not run:\n' + r.stdout[-1500:]
    n_stitch = int(m.group(1))
    with open(evidence(out), encoding='utf-8') as f:
        assert f.read().count('(filled_polygon') == 1, \
            'the GND zone\'s stored fill is gone from the output -- the KiCad leg tests nothing'
    new = _new_vias(src, out)
    assert len(new) == n_stitch, f'{len(new)} new via blocks, the step reports {n_stitch}'
    not_free = [b for b in new if '(free yes)' not in b]
    assert not not_free, (f'{len(not_free)} of {len(new)} stitch vias are written without '
                          f'(free yes); KiCad re-nets each from the stored fill it sits in:\n'
                          + not_free[0])
    wrong = [b for b in new if '(net "/VCC")' not in b]
    assert not wrong, f'{len(wrong)} stitch vias are not written /VCC:\n' + wrong[0]
    pcb = parse_kicad_pcb(out)
    ids = {_uuid(b) for b in new}
    parsed = [v for v in pcb.vias if v.uuid in ids]
    assert len(parsed) == len(new) and all(v.free for v in parsed) and \
        {pcb.nets[v.net_id].name for v in parsed} == {'/VCC'}, \
        'the parser does not read the stitch vias as free /VCC vias'
    print(f'  ok    {len(new)} stitch vias, every one (free yes) and /VCC in the file and the parser')

    print('\n2. the token follows the file format')
    from kicad_writer import generate_via_sexpr, FREE_VIA_BOOL_MIN_VERSION
    for ver, want in ((20221018, '(free)'), (FREE_VIA_BOOL_MIN_VERSION - 1, '(free)'),
                      (FREE_VIA_BOOL_MIN_VERSION, '(free yes)'), (20260206, '(free yes)'),
                      (None, '(free yes)')):
        s = generate_via_sexpr(1, 2, 0.6, 0.3, ['F.Cu', 'B.Cu'], 1, True,
                               net_name='/VCC', file_version=ver)
        assert want in s and ('(free yes)' not in s if want == '(free)' else True), \
            f'file version {ver}: expected {want}, got:\n{s}'
        vias = extract_vias(f'(kicad_pcb (version {ver or 20260206})\n{s}\n)', {'/VCC': 1})
        assert len(vias) == 1 and vias[0].free, f'the parser does not read {want} as free'
    plain = generate_via_sexpr(1, 2, 0.6, 0.3, ['F.Cu', 'B.Cu'], 1, False,
                               net_name='/VCC', file_version=20221018)
    assert '(free' not in plain and not extract_vias(f'(kicad_pcb\n{plain}\n)', {'/VCC': 1})[0].free
    print('  ok    (free) before the KiCad 8 format, (free yes) from it; the parser reads both')

    print('\n3. control: the vias a route step adds are not free')
    rout = os.path.join(d, 'routed.kicad_pcb')
    check(py + [tool('route.py'), src, rout, '--nets', '/Q1', '/OUT_*', '/CLK*', '/DATA*'],
          accept=True)
    rnew = _new_vias(src, rout)
    assert rnew, 'route.py added no vias on this board -- the control tests nothing'
    assert not [b for b in rnew if '(free' in b], 'route.py wrote a routed via free'
    print(f'  ok    {len(rnew)} routed vias, none free')

    print("\n4. KiCad's load keeps the stitch vias' net")
    from kicad_exact_fill import find_kicad_python
    kpy = find_kicad_python()
    if kpy is None:
        print("SKIP: KiCad's bundled python not found (1-3 passed)")
        return SKIP_EXIT
    want = os.path.join(d, 'stitch_uuids.json')
    with open(want, 'w', encoding='utf-8') as f:
        json.dump(sorted(ids), f)
    r = subprocess.run([kpy, '-c', PROBE, out, want], capture_output=True, text=True,
                       timeout=600)
    row = next((json.loads(ln[len('RESULT '):]) for ln in r.stdout.splitlines()
                if ln.startswith('RESULT ')), None)
    if r.returncode != 0 or row is None or len(row) != len(ids):
        print(f'BROKEN TEST: the pcbnew probe read {len(row or {})} of {len(ids)} vias '
              f'(exit {r.returncode}):\n{r.stdout[-2000:]}\n{r.stderr[-3000:]}')
        return 2
    nets = {}
    for n in row.values():
        nets[n] = nets.get(n, 0) + 1
    assert nets == {'/VCC': len(ids)}, \
        f'pcbnew.LoadBoard reads the stitch vias as {nets}, the file writes /VCC for all'
    print(f'  ok    pcbnew.LoadBoard reads all {len(ids)} stitch vias as /VCC')
    print('\nPASS #2067: stitch vias are written free and keep their net in KiCad')
    return 0


if __name__ == '__main__':
    sys.exit(main())
