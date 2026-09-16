#!/usr/bin/env python3
"""#972: the provenance ledger chains on the ARRANGEMENT, not on file bytes.

`provenance_audit` took pose claims only from rows naming the delivered file,
so a declared write to a NEW path turned an earlier hand edit into
`unverifiable` and graded CLEAN. The fix links rows through a pose digest --
`parent_pose_sha256` -> an earlier row's `board_pose_sha256` -- back to the
staged board. This file pins both halves:

  1. the digest itself: what it must NOT see (a byte copy, `stamp_locked`,
     -90 vs 270, `(at x y)` vs `(at x y 0)`, a `.6g` rotation re-emitted) and
     what it MUST see (a rotation, a side flip, a padless footprint's move),
     plus the row keys, the redaction, and that it never raises and never runs
     outside a regime;

It is a separate file from `test_provenance_audit.py` on purpose: that file
runs route.py's `__main__` through `runpy`, which leaves `route.py` DECLARED
for the rest of its process, and every case here depends on what is and is
not declared.
"""
import contextlib
import io
import os
import re
import shutil
import sys
import tempfile

REPO = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
for _p in ('', 'py_router', 'py_tools', 'py_placer', os.path.join('tests', 'stress')):
    _q = os.path.join(REPO, _p)
    if _q not in sys.path:
        sys.path.insert(0, _q)

SF = os.path.join(REPO, 'kicad_files', 'splitflap_driver.kicad_pcb')
EP = os.path.join(REPO, 'kicad_files', 'esp_prog.kicad_pcb')

passed = failed = 0


def check(name, ok, detail=''):
    global passed, failed
    passed += bool(ok)
    failed += not ok
    print(f"  {'OK  ' if ok else 'FAIL'} {name}{(' -- ' + detail) if detail else ''}")


if not (os.path.isfile(SF) and os.path.isfile(EP)):
    print('SKIP: fixture missing')
    sys.exit(77)

from kicad_parser import parse_kicad_pcb                      # noqa: E402
from placement import provenance as PV                        # noqa: E402
from placement.legality import footprint_side                 # noqa: E402
from placement.writer import write_placed_output              # noqa: E402
from placement.seeder import stamp_locked                     # noqa: E402

_sink = io.StringIO()


def quiet(f, *a, **k):
    with contextlib.redirect_stdout(_sink), contextlib.redirect_stderr(_sink):
        return f(*a, **k)


def fresh(board=SF):
    """A staged, armed work dir. Returns (workdir, staged board)."""
    d = tempfile.mkdtemp(prefix='t972_')
    wd = os.path.join(d, 'wk')
    os.makedirs(wd)
    staged = os.path.join(wd, 'board.kicad_pcb')
    shutil.copyfile(board, staged)
    PV.start_regime(wd, staged)
    return wd, staged


def outside():
    """A directory no regime governs -- where a hand edit is authored."""
    return tempfile.mkdtemp(prefix='t972_out_')


def fp_of(path, ref):
    return parse_kicad_pcb(path).footprints[ref]


def hand_edit(board, ref, dx=9.0, dy=5.0, rot=None, into=None):
    """Move `ref` with no lever and no row: written outside every regime, then
    copied over `into` (default: `board` itself)."""
    fp = fp_of(board, ref)
    tmp = os.path.join(outside(), 'h.kicad_pcb')
    quiet(write_placed_output, board, tmp,
          [{'reference': ref, 'new_x': fp.x + dx, 'new_y': fp.y + dy,
            'new_rotation': fp.rotation if rot is None else rot}])
    shutil.copyfile(tmp, into or board)


def rows(wd):
    return PV.read_ledger(wd)


def settled(tag):
    check(f'{tag}: nothing pending, no lever left declared',
          PV._PENDING == {} and PV.active_lever() is None,
          f'pending={list(PV._PENDING)} lever={PV.active_lever()}')


def independent_digest(path):
    """The canonical form, re-derived here from `parse_kicad_pcb` rather than
    from `PV.pose_table`: a test that computes both sides with the helper under
    test can only ever agree with it."""
    import hashlib
    import json
    t = []
    for ref, f in sorted(parse_kicad_pcb(path).footprints.items()):
        rot = f.rotation or 0.0
        t.append([ref, round(f.x * 1e6), round(f.y * 1e6),
                  round((rot % 360.0) * 1e4) % 3600000, footprint_side(f)])
    blob = json.dumps(t, separators=(',', ':'), ensure_ascii=True)
    return 'p1:' + hashlib.sha256(blob.encode('ascii')).hexdigest()


# ==========================================================================
# 1. the digest
# ==========================================================================
print('1. the pose digest')

D = PV.pose_digest
base_t = {'A': (1.0, 2.0, 0.0, 'F'), 'B': (3.0, 4.0, 90.0, 'F')}
check('-90 and 270 are one rotation',
      D({'A': (1.0, 2.0, -90.0, 'F')}) == D({'A': (1.0, 2.0, 270.0, 'F')}))
check('360 and 0 are one rotation',
      D({'A': (1.0, 2.0, 360.0, 'F')}) == D({'A': (1.0, 2.0, 0.0, 'F')}))
check('-0.0 and 0.0 are one coordinate',
      D({'A': (-0.0, 2.0, -0.0, 'F')}) == D({'A': (0.0, 2.0, 0.0, 'F')}))
check('a -1e-17 rotation (which `% 360` makes 360.0) is 0',
      D({'A': (1.0, 2.0, -1e-17, 'F')}) == D({'A': (1.0, 2.0, 0.0, 'F')}))
check('sub-nanometre float noise does not split a pose',
      D({'A': (1.0000000001, 2.0, 0.0, 'F')}) == D({'A': (1.0, 2.0, 0.0, 'F')}))
check('dict order does not matter',
      D(base_t) == D(dict(reversed(list(base_t.items())))))
check('ONE nanometre is a different arrangement',
      D({'A': (1.000001, 2.0, 0.0, 'F')}) != D({'A': (1.0, 2.0, 0.0, 'F')}))
check('a side flip alone is a different arrangement',
      D({'A': (1.0, 2.0, 0.0, 'B')}) != D({'A': (1.0, 2.0, 0.0, 'F')}))
check('a removed footprint is a different arrangement',
      D({'A': base_t['A']}) != D(base_t))
check('the value carries its scheme', D(base_t).startswith('p1:'))

_ep_digest = PV.file_pose_digest(EP)
# GOLDEN. If this moves, the canonical form moved, and every digest in every
# existing ledger stops linking: bump POSE_DIGEST_SCHEME instead of editing
# this literal, so an old ledger reads as a different scheme rather than as a
# broken chain.
check('esp_prog digest is the golden literal',
      _ep_digest == 'p1:f705b12de8118183a39eac2f1301c04c927b011c939e5ef64b6b81c74829c866',
      str(_ep_digest))
check('...and an independent derivation from parse_kicad_pcb agrees',
      _ep_digest == independent_digest(EP))
_pt, _pk = PV.pose_table(EP), parse_kicad_pcb(EP).footprints
check('pose_table keys are parse_kicad_pcb keys (Ref*~2, #uuid included)',
      set(_pt) == set(_pk) and any('~' in r for r in _pt)
      and any(r.startswith('#') for r in _pt),
      f'{sorted(set(_pt) ^ set(_pk))}')

# --- rewrites that move nothing ------------------------------------------
_o = outside()
_copy = os.path.join(_o, 'copy.kicad_pcb')
shutil.copyfile(SF, _copy)
check('a byte copy has the same digest', PV.file_pose_digest(_copy) == PV.file_pose_digest(SF))

_locked = os.path.join(_o, 'locked.kicad_pcb')
shutil.copyfile(SF, _locked)
_lk_ref = sorted(parse_kicad_pcb(SF).footprints)[0]
_n_locked = quiet(stamp_locked, _locked, [_lk_ref])
check('stamp_locked changes the BYTES...',
      _n_locked == 1 and PV.sha256_file(_locked) != PV.sha256_file(SF),
      f'stamped {_n_locked}')
check('...and not the arrangement', PV.file_pose_digest(_locked) == PV.file_pose_digest(SF))

# `(at x y)` and `(at x y 0)` are one pose. Rewrite the FIRST footprint's own
# `(at` into the other spelling, in text, so no writer normalises it back.
with open(SF, encoding='utf-8') as _f:
    _txt = _f.read()
_m = re.search(r'\(footprint\s+"[^"]*"(?:(?!\(footprint\s).)*?\(at\s+([\d.-]+)\s+([\d.-]+)(\s+[\d.-]+)?\)',
               _txt, flags=re.S)
if _m.group(3) is None:                          # (at x y)   -> (at x y 0)
    _spelt = _txt[:_m.end(2)] + ' 0' + _txt[_m.end(2):]
elif float(_m.group(3)) == 0:                     # (at x y 0) -> (at x y)
    _spelt = _txt[:_m.start(3)] + _txt[_m.end(3):]
else:                                             # (at x y r) -> (at x y r-360)
    _spelt = (_txt[:_m.start(3)] + ' ' + str(float(_m.group(3)) - 360.0)
              + _txt[_m.end(3):])
_sp = os.path.join(_o, 'spelt.kicad_pcb')
with open(_sp, 'w', encoding='utf-8', newline='') as _f:
    _f.write(_spelt)
check('a respelt `(at ...)` (implicit 0, or -360 offset) has the same digest',
      _spelt != _txt and PV.file_pose_digest(_sp) == PV.file_pose_digest(SF))

# A `.6g` rotation: the writer emits 137.253 for 137.253491, and a later
# identity write re-reads and re-emits that text.
_r6a = os.path.join(_o, 'r6a.kicad_pcb')
_r6b = os.path.join(_o, 'r6b.kicad_pcb')
_fp0 = fp_of(SF, _lk_ref)
quiet(write_placed_output, SF, _r6a, [{'reference': _lk_ref, 'new_x': _fp0.x,
                                       'new_y': _fp0.y, 'new_rotation': 137.253491}])
quiet(write_placed_output, _r6a, _r6b, [])
check('a .6g rotation survives an identity rewrite', PV.file_pose_digest(_r6a) == PV.file_pose_digest(_r6b))

# --- rewrites that DO move something -------------------------------------
_rot = os.path.join(_o, 'rot.kicad_pcb')
quiet(write_placed_output, SF, _rot, [{'reference': _lk_ref, 'new_x': _fp0.x,
                                       'new_y': _fp0.y, 'new_rotation': (_fp0.rotation or 0) + 0.01}])
check('a 0.01 degree rotation is seen', PV.file_pose_digest(_rot) != PV.file_pose_digest(SF))
_flip = os.path.join(_o, 'flip.kicad_pcb')
quiet(write_placed_output, SF, _flip, [{'reference': _lk_ref, 'new_x': _fp0.x, 'new_y': _fp0.y,
                                        'new_rotation': _fp0.rotation, 'new_side': 'B'}])
check('a flip in place is seen', PV.file_pose_digest(_flip) != PV.file_pose_digest(SF))

# A PADLESS footprint (esp_prog's reference-less logo blocks). A digest built
# from pad-bearing parts only would miss it; move it in raw text.
_padless = next(r for r, f in PV.pose_footprints(EP).items() if not f.pads)
with open(EP, encoding='utf-8') as _f:
    _etxt = _f.read()
from kicad_parser import iter_footprint_blocks                # noqa: E402
_blk = next(b for b in iter_footprint_blocks(_etxt) if b[4] == _padless)
_bt = _blk[2]
_am = re.search(r'\(at\s+([\d.-]+)\s+([\d.-]+)', _bt)
_nb = _bt[:_am.start(1)] + f'{float(_am.group(1)) + 3.0:.6f}' + _bt[_am.end(1):]
_pl = os.path.join(_o, 'padless.kicad_pcb')
with open(_pl, 'w', encoding='utf-8', newline='') as _f:
    _f.write(_etxt[:_blk[0]] + _nb + _etxt[_blk[1]:])
check('a padless footprint moved by hand is seen',
      PV.file_pose_digest(_pl) != _ep_digest, _padless)

check('file_pose_digest of a missing file is None, not an exception',
      PV.file_pose_digest(os.path.join(_o, 'nope.kicad_pcb')) is None)
_junk = os.path.join(_o, 'junk.kicad_pcb')
with open(_junk, 'wb') as _f:
    _f.write(b'\xff\xfe not a board')
check('file_pose_digest of an unreadable file is None, not an exception',
      PV.file_pose_digest(_junk) is None)


# ==========================================================================
# 2. the row keys
# ==========================================================================
print('2. the row keys')

wd, staged = fresh()
_refs = sorted(parse_kicad_pcb(staged).footprints)[:3]
A = os.path.join(wd, 'A.kicad_pcb')
_mv = [{'reference': r, 'new_x': 12.0 + i, 'new_y': 34.0, 'new_rotation': 0.0}
       for i, r in enumerate(_refs[:2])]
_staged_digest = PV.file_pose_digest(staged)
with PV.declare_lever('place_optimize.py'):
    quiet(write_placed_output, staged, A, _mv)
_r = rows(wd)
check('an engine row carries both pose digests',
      len(_r) == 1 and 'parent_pose_sha256' in _r[0] and 'board_pose_sha256' in _r[0],
      f'{[sorted(k for k in r if "sha" in k) for r in _r]}')
check('parent_pose_sha256 is the INPUT arrangement',
      _r and _r[0]['parent_pose_sha256'] == _staged_digest == independent_digest(staged))
check('board_pose_sha256 is the arrangement of the FILE written',
      _r and _r[0]['board_pose_sha256'] == PV.file_pose_digest(A) == independent_digest(A)
      and _r[0]['board_pose_sha256'] != _staged_digest)

_before_inplace = PV.file_pose_digest(A)
with PV.declare_lever('place_optimize.py'):
    quiet(write_placed_output, A, A, [{'reference': _refs[2], 'new_x': 50.0,
                                       'new_y': 60.0, 'new_rotation': 90.0}])
_r = rows(wd)
check('an IN-PLACE write hashes the board it replaces as its parent',
      len(_r) == 2 and _r[1]['parent_pose_sha256'] == _before_inplace
      and _r[1]['board_pose_sha256'] == PV.file_pose_digest(A) != _before_inplace)
check('...which is the previous row\'s board digest (the link)',
      len(_r) == 2 and _r[1]['parent_pose_sha256'] == _r[0]['board_pose_sha256'])

# stamp_locked between two rows: the BYTE chain breaks, the POSE chain holds.
stamp_locked(A, [_refs[0]])
B = os.path.join(wd, 'B.kicad_pcb')
with PV.declare_lever('place_optimize.py'):
    quiet(write_placed_output, A, B, [])
_r = rows(wd)
check('after stamp_locked the byte parent matches no recorded board...',
      _r[2]['parent_sha256'] not in {x.get('board_sha256') for x in _r[:2]})
check('...and the pose parent still links',
      _r[2]['parent_pose_sha256'] == _r[1]['board_pose_sha256'])

# A staging row: neither digest, and none of the keys redaction already drops.
with PV.declare_lever('stage_unaided.py'):
    quiet(write_placed_output, staged, os.path.join(wd, 'restage.kicad_pcb'), _mv)
_st = rows(wd)[-1]
_leak = {'parent_pose_sha256', 'board_pose_sha256', 'parent_sha256', 'lever_argv',
         'poses_written', 'refs_written', 'refs_moved', 'sides_written'} & set(_st)
check('a staging row is redacted and carries no pose digest',
      'redacted' in _st and not _leak, f'leaked {sorted(_leak)}')
settled('rows')

# --- the digest never raises into the writer ------------------------------
wd2, staged2 = fresh()
_real = PV.pose_footprints


def _boom(path):
    raise RuntimeError('injected parse failure')


PV.pose_footprints = _boom
try:
    with PV.declare_lever('place_optimize.py'):
        quiet(write_placed_output, staged2, os.path.join(wd2, 'X.kicad_pcb'), _mv)
    _raised = None
except Exception as e:                                         # noqa: BLE001
    _raised = e
finally:
    PV.pose_footprints = _real
_r = rows(wd2)
check('a parse failure does not stop the write or the row',
      _raised is None and len(_r) == 1 and os.path.isfile(os.path.join(wd2, 'X.kicad_pcb')),
      repr(_raised))
check('...and records the digests as unknown, not as a value',
      _r and _r[0].get('parent_pose_sha256') is None and _r[0].get('board_pose_sha256') is None
      and 'parent_pose_sha256' in _r[0] and 'board_pose_sha256' in _r[0])
settled('parse failure')

# --- no digest work outside a regime -------------------------------------
_calls = []


def _count(path):
    _calls.append(path)
    return _real(path)


PV.pose_footprints = _count
try:
    _free = outside()
    quiet(write_placed_output, SF, os.path.join(_free, 'free.kicad_pcb'), _mv)
finally:
    PV.pose_footprints = _real
check('outside a regime the writer parses nothing for provenance', _calls == [],
      f'{len(_calls)} call(s)')
settled('outside')


print(f'\n{passed} passed, {failed} failed')
sys.exit(1 if failed else 0)
