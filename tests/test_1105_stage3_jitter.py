#!/usr/bin/env python3
"""#1105: seeder stage 3's jitter is drawn per queue entry before any reorder.

Stage 3.5 (`--decap-claim-after-ics`) seats a claimed cap at a supply pin, so
that cap skips its centroid turn; its `after_queue` variant also moves the
caps to the end of the queue. The centroid seat used to draw its target
jitter at the turn, so either one shifted the RNG stream for every part
seated after it, and the stage's A/B rows measured a re-roll of those
targets as well as the claim. What each case pins:

* every ref the centroid seat places in BOTH arms gets the same jitter with
  the stage on as with it off, under both `DECAP_LATE_AT` modes, on esp_prog
  and watchy; and the stage did claim caps (anti-vacuity).
* with the stage off the seed is the one the pre-#1105 seeder made
  (tests/fixtures/1105/seed_off_baseline.json, the same fixture
  test_1105_decap_claim_after_ics.py reads), so drawing the jitter up front
  changed no pose.
* the same with anchors-first on (tests/fixtures/1105/
  seed_anchors_first_baseline.json, recorded at 055fa9e1 with `--record`).
  None of the fixture's other runs turns anchors-first on, so a draw moved
  ABOVE the anchors-first reorder changed every pose of such a seed (18 of 18
  on esp_prog, 84 of 84 on watchy) with every other case green.

    python3 tests/test_1105_stage3_jitter.py [name-substring ...]
    python3 tests/test_1105_stage3_jitter.py --record <repo root>
"""
import json
import os
import random
import sys
import tempfile

TESTS_DIR = os.path.dirname(os.path.abspath(__file__))
#: `--record <root>` seeds with ANOTHER checkout's code (one before the
#: jitter fix) and rewrites the anchors-first fixture from it.
_REC = (sys.argv.index('--record')
        if '--record' in sys.argv[1:] else None)
ROOT = (os.path.abspath(sys.argv[_REC + 1]) if _REC is not None
        else os.path.dirname(TESTS_DIR))
ANCHORS_BASELINE = os.path.join(TESTS_DIR, 'fixtures', '1105',
                                'seed_anchors_first_baseline.json')
_ANCHOR_RUNS = (('esp_prog', '0'), ('watchy', '0'))
for _d in ('py_router', 'py_placer', 'py_tools'):
    sys.path.insert(0, os.path.join(ROOT, _d))
sys.path.insert(0, TESTS_DIR)

from kicad_parser import parse_kicad_pcb           # noqa: E402
from placement import floorplan as fp              # noqa: E402
from placement import seeder                       # noqa: E402

RUN_ALL_TIMEOUT = 1800

BOARDS = os.path.join(ROOT, 'kicad_files')
ESP = os.path.join(BOARDS, 'esp_prog.kicad_pcb')
WATCHY = os.path.join(BOARDS, 'watchy.kicad_pcb')
SOURCES = ('kicad', 'sheet')
CLEARANCE = 0.2
LIMIT = 3.0


def _intent(board, td):
    """The board's own intent, armed with a 3 mm decap limit (as
    test_1105_decap_claim_after_ics.py arms it)."""
    doc = fp.emit_intent(parse_kicad_pcb(board), board)
    doc['decaps'] = dict(doc.get('decaps') or {}, max_distance_mm=LIMIT)
    path = os.path.join(td, os.path.basename(board) + '.json')
    with open(path, 'w', encoding='utf-8') as fh:
        json.dump(doc, fh, indent=1)
    return fp.load_intent(path)


def _seed(board, intent, seed='0', **kw):
    pcb = parse_kicad_pcb(board)
    return seeder.seed_from_intent(
        pcb, board, intent, random.Random(seed), group_sources=SOURCES,
        clearance=CLEARANCE, **kw)


class _knobs:
    """Set seeder module knobs for one block, and restore them."""

    def __init__(self, **kw):
        self.kw, self.saved = kw, {}

    def __enter__(self):
        for k, v in self.kw.items():
            self.saved[k] = getattr(seeder, k)
            setattr(seeder, k, v)

    def __exit__(self, *exc):
        for k, v in self.saved.items():
            setattr(seeder, k, v)
        return False


class _jitter_log:
    """Record the jitter every centroid seat aims with: wrap `_try_place` and
    read `jx`/`jy` off the calling frame when the caller is `_centroid_seat`.
    Reads the value the seat USED, not one it reports."""

    def __init__(self):
        self.got = {}

    def __enter__(self):
        real = self.real = seeder._try_place
        got = self.got

        def spy(state, ref, *a, **k):
            f = sys._getframe(1)
            if f.f_code.co_name == '_centroid_seat':
                got.setdefault(ref, (f.f_locals['jx'], f.f_locals['jy']))
            return real(state, ref, *a, **k)
        seeder._try_place = spy
        return self

    def __exit__(self, *exc):
        seeder._try_place = self.real
        return False


def test_stage_3_jitter_is_the_same_with_the_claim_on_or_off():
    with tempfile.TemporaryDirectory() as td:
        n = {}
        for board in (ESP, WATCHY):
            intent = _intent(board, td)
            for at in ('after_last_owner', 'after_queue'):
                with _knobs(DECAP_LATE_AT=at, DECAP_LATE_WITHIN_LIMIT=False):
                    with _jitter_log() as off:
                        _seed(board, intent, decap_claim_after_ics=False)
                    with _jitter_log() as on:
                        res = _seed(board, intent,
                                    decap_claim_after_ics=True)
                late = res['decap_stage']['late']
                # anti-vacuity: the stage claimed caps, so those caps skipped
                # their centroid turn -- the case the draw order is about.
                assert late['claimed'] > 0, (board, at, late)
                assert set(late['caps']) - set(on.got), (board, at)
                both = sorted(set(off.got) & set(on.got))
                assert len(both) >= 5, (board, at, both)
                diff = [r for r in both if off.got[r] != on.got[r]]
                assert not diff, (board, at, diff[:5])
                n[(os.path.basename(board), at)] = len(both)
    print(f"  PASS: identical jitter OFF vs ON for every centroid-seated "
          f"ref: {n}")


def test_the_stage_off_is_still_the_pre_1105_seeder():
    """Drawing stage 3's jitter up front must not move a pose: the stage-off
    seed is test_1105's fixture, recorded before stage 3.5 existed."""
    base_path = os.path.join(TESTS_DIR, 'fixtures', '1105',
                             'seed_off_baseline.json')
    with open(base_path, encoding='utf-8') as fh:
        base = json.load(fh)
    import test_1105_decap_claim_after_ics as t1105
    n = 0
    with tempfile.TemporaryDirectory() as td:
        for board, seed in t1105._OFF_RUNS:
            want = base['runs'][f'{board}:{seed}']
            got = t1105._off_run(board, seed, td, decap_claim_after_ics=False)
            assert got['placements'] == want['placements'], (board, seed)
            assert got['notes'] == want['notes'], (board, seed)
            n += len(got['placements'])
    print(f"  PASS: {n} stage-off placements identical to the seeder at "
          f"{base['recorded_at']}")


def _anchors_run(board, seed, td):
    import test_1105_decap_claim_after_ics as t1105
    r = t1105._off_run(board, seed, td, decap_claim_after_ics=False,
                       anchors_first=True)
    return {k: r[k] for k in ('placements', 'unseated', 'notes')}


def test_anchors_first_with_the_stage_off_is_unchanged():
    """The draw sits AFTER the anchors-first reorder, so an anchors-first
    seed's entries draw in the queue order they are seated in, exactly as the
    inline draw did."""
    with open(ANCHORS_BASELINE, encoding='utf-8') as fh:
        base = json.load(fh)
    n = 0
    with tempfile.TemporaryDirectory() as td:
        for board, seed in _ANCHOR_RUNS:
            want = base['runs'][f'{board}:{seed}']
            got = _anchors_run(board, seed, td)
            # anti-vacuity: anchors-first ran and reordered something
            assert any(s.startswith('anchors-first:') for s in got['notes']), (
                board, got['notes'][:5])
            assert got['placements'] == want['placements'], (
                board, seed, sorted(r for r in got['placements']
                                    if got['placements'][r]
                                    != want['placements'].get(r)))
            assert got['unseated'] == want['unseated'], (board, seed)
            assert got['notes'] == want['notes'], (board, seed)
            n += len(got['placements'])
    print(f"  PASS: {n} anchors-first stage-off placements identical to the "
          f"seeder at {base['recorded_at']}")


def _record():
    """Rewrite the anchors-first fixture with the code under `--record`."""
    import subprocess
    sha = subprocess.run(['git', '-C', ROOT, 'rev-parse', '--short=8',
                          'HEAD'], capture_output=True, text=True).stdout.strip()
    runs = {}
    with tempfile.TemporaryDirectory() as td:
        for board, seed in _ANCHOR_RUNS:
            runs[f'{board}:{seed}'] = _anchors_run(board, seed, td)
    with open(ANCHORS_BASELINE, 'w', encoding='utf-8', newline='\n') as fh:
        json.dump({'recorded_at': sha, 'root': os.path.basename(ROOT),
                   'clearance': CLEARANCE, 'limit_mm': LIMIT, 'runs': runs},
                  fh, indent=1, sort_keys=True)
        fh.write('\n')
    print(f"wrote {ANCHORS_BASELINE} from {ROOT} at {sha}")


TESTS = [
    test_stage_3_jitter_is_the_same_with_the_claim_on_or_off,
    test_the_stage_off_is_still_the_pre_1105_seeder,
    test_anchors_first_with_the_stage_off_is_unchanged,
]


if __name__ == '__main__':
    if _REC is not None:
        _record()
        sys.exit(0)
    only = sys.argv[1:]
    ran = 0
    for t in TESTS:
        if only and not any(o in t.__name__ for o in only):
            continue
        print(f"--- {t.__name__}")
        t()
        ran += 1
    if only and not ran:
        # A filter that names no case passes nothing: a mutation battery
        # witness spelled wrong would otherwise read every row as SURVIVED.
        print(f"NO TEST matches {only}")
        sys.exit(2)
    print('ALL PASS')
