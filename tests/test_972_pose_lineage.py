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
  2. the lineage `provenance_audit` walks with it: every laundering shape a
     review could build (a write to a new path, in place, a no-op, two no-ops
     vouching for each other, a write-all pass-through, a splice of two
     candidates, a revert, a row whose file disagrees with its claims, a
     staging lever, malformed rows, an old staging epoch) must NOT grade
     CLEAN, and the legitimate flows beside them (routed copper and an
     in-place cap move, a lock stamp, a flip, place_seed's `.polish` rename, a
     hand-added part) must -- plus the pre-digest (`legacy`) and unlinkable
     readings.

The literal #972 sequence on esp_prog, and its clean control, live in
`test_provenance_audit.py` as the issue asks.

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
# esp_prog's angles are all multiples of 90 and its refs are ASCII, so the
# literal above cannot see the rounding step or the escaping. This one can:
# 137.253 * 1e4 is 1372529.99..., which `int()` would truncate, and a non-ASCII
# ref spells differently with `ensure_ascii=False`.
check('a synthetic table with a .6g angle and a non-ASCII ref is its golden literal',
      D({'C1': (1.5, -2.25, 137.253, 'F'), 'Ω1': (0.0, 0.0, -90.0, 'B'),
         'R2': (10.0000004, 3.3333333, 0.00005, 'F')})
      == 'p1:ee01be11cb4a0ddf021b5633c5b9bc90192d5888a27cfa2d1a607279dbd1793d')
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

# A `.6g` rotation: the writer emits 137.253 for 137.253491, and a later write
# that RE-EMITS every part at the pose it parsed must land on the same digest.
# (An empty write is byte-identical, which would make this check unable to
# fail.)
_r6a = os.path.join(_o, 'r6a.kicad_pcb')
_r6b = os.path.join(_o, 'r6b.kicad_pcb')
_fp0 = fp_of(SF, _lk_ref)
quiet(write_placed_output, SF, _r6a, [{'reference': _lk_ref, 'new_x': _fp0.x,
                                       'new_y': _fp0.y, 'new_rotation': 137.253491}])
quiet(write_placed_output, _r6a, _r6b,
      [{'reference': r, 'new_x': p[0], 'new_y': p[1], 'new_rotation': p[2]}
       for r, p in PV.pose_table(_r6a).items()])
check('a .6g rotation survives a rewrite of every part at its parsed pose',
      PV.sha256_file(_r6a) != PV.sha256_file(_r6b)
      and PV.file_pose_digest(_r6a) == PV.file_pose_digest(_r6b),
      f'bytes differ: {PV.sha256_file(_r6a) != PV.sha256_file(_r6b)}')

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

# The input PARSED, but digesting it failed. That is the parent digest's own
# guard, which the injection above never reaches (a failed parse leaves no
# footprints to digest).
wd3, staged3 = fresh()
_real_of = PV.pose_table_of


def _boom_of(fps):
    raise RuntimeError('injected digest failure')


PV.pose_table_of = _boom_of
try:
    with PV.declare_lever('place_optimize.py'):
        quiet(write_placed_output, staged3, os.path.join(wd3, 'Y.kicad_pcb'), _mv)
    _raised = None
except Exception as e:                                         # noqa: BLE001
    _raised = e
finally:
    PV.pose_table_of = _real_of
_r = rows(wd3)
check('a digest failure after a good parse does not stop the write or the row',
      _raised is None and len(_r) == 1 and _r[0].get('parent_pose_sha256') is None
      and _r[0].get('refs_moved') == sorted(m['reference'] for m in _mv),
      repr(_raised))
settled('digest failure')

# The NON-pending path stamps the board digest too. No production lever calls
# it today; a future one must not get rows the lineage cannot link.
wd4, staged4 = fresh()
_B4 = os.path.join(wd4, 'B.kicad_pcb')
shutil.copyfile(staged4, _B4)
with PV.declare_lever('place_optimize.py'):
    _row4 = PV.record_write(staged4, _B4, [])
check('a direct (non-pending) row carries the written board\'s digest',
      _row4 is not None and _row4.get('board_pose_sha256') == PV.file_pose_digest(_B4)
      and rows(wd4) and rows(wd4)[0].get('board_pose_sha256') == PV.file_pose_digest(_B4),
      str(_row4 and _row4.get('board_pose_sha256')))
settled('direct row')

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


# ==========================================================================
# 3. the lineage: laundering attempts, and the legitimate flows beside them
# ==========================================================================
print('3. the lineage')
import json                                                    # noqa: E402
import subprocess                                              # noqa: E402
import provenance_audit as PA                                  # noqa: E402

_REFS = sorted(parse_kicad_pcb(SF).footprints)
X, Y, Z = _REFS[0], _REFS[1], _REFS[2]


def mv(board, ref, dx=2.0, dy=0.0, rot=None, side=None):
    fp = fp_of(board, ref)
    p = {'reference': ref, 'new_x': fp.x + dx, 'new_y': fp.y + dy,
         'new_rotation': fp.rotation if rot is None else rot}
    if side is not None:
        p['new_side'] = side
    return p


def lever_write(inp, out, placements, lever='place_optimize.py'):
    with PV.declare_lever(lever):
        quiet(write_placed_output, inp, out, placements)


def grade(tag, wd, board, want_code, **want):
    code, doc = PA.audit(wd, board)
    ok = code == want_code and all(doc.get(k) == v for k, v in want.items())
    check(tag, ok, f"exit {code} {doc.get('verdict')} lineage={doc.get('lineage')} "
                   f"drifted={doc.get('drifted_refs')} unclaimed={doc.get('unclaimed_refs')} "
                   f"unverifiable={doc.get('unverifiable_claims')}")
    return code, doc


def rewrite_ledger(wd, fn):
    path = os.path.join(wd, PV.LEDGER_NAME)
    rs = PV.read_ledger(wd)
    with open(path, 'w', encoding='utf-8') as f:
        for r in rs:
            r = fn(r)
            if r is not None:
                f.write(json.dumps(r, sort_keys=True) + '\n')


def s972():
    """The #972 shape on splitflap: E1 moves X into A, X is hand-edited in A.
    Returns (wd, staged, A)."""
    wd, st = fresh()
    A = os.path.join(wd, 'A.kicad_pcb')
    lever_write(st, A, [mv(st, X)])
    hand_edit(A, X)
    return wd, st, A


# --- laundering attempts: none may grade CLEAN ----------------------------
wd, st, A = s972()
F = os.path.join(wd, 'final.kicad_pcb')
lever_write(A, F, [mv(A, Y)])
grade('V1 #972 on splitflap: write to a new path names X', wd, F, PA.VIOLATION,
      drifted_refs=[X], lineage='broken', unverifiable_claims=[])

wd, st, A = s972()
lever_write(A, A, [mv(A, Y)])
grade('V2 the same write IN PLACE names X, not the ref it moved', wd, A, PA.VIOLATION,
      drifted_refs=[X], lineage='broken')

wd, st = fresh()
a, b = os.path.join(wd, 'a.kicad_pcb'), os.path.join(wd, 'b.kicad_pcb')
lever_write(st, a, [mv(st, X)])
lever_write(st, b, [mv(st, Y)])
S = os.path.join(wd, 'S.kicad_pcb')
_t = os.path.join(outside(), 's.kicad_pcb')
quiet(write_placed_output, st, _t, [mv(st, X), mv(st, Y)])
shutil.copyfile(_t, S)
# Both candidates differ from the splice by one part; the tie goes to the most
# recent state (b), so the part taken from a is the one named. Deterministic.
grade('V4 a hand splice of two recorded candidates is a VIOLATION', wd, S, PA.VIOLATION,
      drifted_refs=[X], lineage='unrecorded')

wd, st, A = s972()
F = os.path.join(wd, 'final.kicad_pcb')
lever_write(A, F, [])
grade('V5 a declared no-op write of the edited board names X', wd, F, PA.VIOLATION,
      drifted_refs=[X], lineage='broken')

wd, st, A = s972()
H2, H3 = os.path.join(wd, 'H2.kicad_pcb'), os.path.join(wd, 'H3.kicad_pcb')
lever_write(A, H2, [])
lever_write(A, H3, [])
grade('V6 two no-op writes cannot vouch for each other', wd, H3, PA.VIOLATION,
      drifted_refs=[X], lineage='broken')

wd, st, A = s972()
F = os.path.join(wd, 'final.kicad_pcb')
_all = [{'reference': r, 'new_x': p[0], 'new_y': p[1], 'new_rotation': p[2]}
        for r, p in PV.pose_table(A).items() if r != Y] + [mv(A, Y)]
lever_write(A, F, _all)
_r = rows(wd)[-1]
check('V7 fixture: the write-all row records X at the HAND pose but does not move it',
      X in _r['poses_written'] and X not in _r['refs_moved'])
grade('V7 a write-all lever passing the hand pose through does not bless it', wd, F,
      PA.VIOLATION, drifted_refs=[X], lineage='broken')

wd, st, A = s972()
F = os.path.join(wd, 'final.kicad_pcb')
lever_write(A, F, [mv(A, X, dx=-4.0, dy=3.0)])
grade('V8 a lever that re-moves the edited part leaves nothing to name: UNPROVEN', wd, F,
      PA.UNPROVEN, lineage='broken', drifted_refs=[], unclaimed_refs=[])

wd, st = fresh()
A = os.path.join(wd, 'A.kicad_pcb')
lever_write(st, A, [mv(st, X)])
H = os.path.join(outside(), 'H.kicad_pcb')
_yv = mv(A, Y)
quiet(write_placed_output, A, H, [_yv])        # exactly what the row will claim...
hand_edit(H, X)                                # ...plus a hand edit it does not
F = os.path.join(wd, 'final.kicad_pcb')
with PV.declare_lever('place_optimize.py'):
    PV.record_write(A, F, [_yv], pending=True)
    shutil.copyfile(H, F)
    PV.commit_write(F)
grade('V9 a row whose delivered file disagrees with its own claims is caught', wd, F,
      PA.VIOLATION, drifted_refs=[X], lineage='verified')

wd, st, A = s972()
F = os.path.join(wd, 'final.kicad_pcb')
lever_write(A, F, [mv(A, Y)])
with open(os.path.join(wd, PV.LEDGER_NAME), 'a', encoding='utf-8') as f:
    f.write(json.dumps({'schema': 1, 'lever': 'place_seed.py', 'declared': True,
                        'path': os.path.join(wd, 'elsewhere.kicad_pcb'),
                        'refs_moved': [], 'poses_written': {}}) + '\n')
grade('V10 one pre-digest row elsewhere in the ledger does not reopen #972', wd, F,
      PA.VIOLATION, drifted_refs=[X], lineage='legacy')

wd, st = fresh()
lever_write(st, st, [], lever='stage_unaided.py')             # a restage: redacted row
PV.start_regime(wd, st)
A = os.path.join(wd, 'A.kicad_pcb')
lever_write(st, A, [mv(st, X)])
hand_edit(A, X)
F = os.path.join(wd, 'final.kicad_pcb')
lever_write(A, F, [mv(A, Y)])
grade('V11 a restaging row is neither a link nor a reason to fall back', wd, F,
      PA.VIOLATION, drifted_refs=[X], lineage='broken')

wd, st = fresh()
A = os.path.join(wd, 'A.kicad_pcb')
lever_write(st, A, [mv(st, X)])
hand_edit(A, Z)                                                # a part NO row claims
F = os.path.join(wd, 'final.kicad_pcb')
lever_write(A, F, [mv(A, Y)])
_c, _d = grade('V12 an unclaimed hand edit carried to a new path is unclaimed', wd, F,
               PA.VIOLATION, unclaimed_refs=[Z], lineage='broken')
check('...and the reason is still the board-not-log one', 'compares the BOARD' in (_d.get('reason') or ''))

wd, st = fresh()
H = os.path.join(outside(), 'H.kicad_pcb')
shutil.copyfile(st, H)
hand_edit(H, X)
F = os.path.join(wd, 'final.kicad_pcb')
lever_write(H, F, [], lever='perturb.py')
grade('V12b a staging lever cannot mint a hand edit', wd, F, PA.VIOLATION, unclaimed_refs=[X])

wd, st, A = s972()
with open(os.path.join(wd, PV.LEDGER_NAME), 'a', encoding='utf-8') as f:
    f.write(json.dumps({'lever': 'place_optimize.py', 'declared': True,
                        'refs_moved': 5, 'path': 'x'}) + '\n')
    f.write('[1, 2, 3]\n')
_p = subprocess.run([sys.executable, '-X', 'utf8', '-B',
                     os.path.join(REPO, 'tests', 'stress', 'provenance_audit.py'),
                     '--workdir', wd, '--delivered', A],
                    capture_output=True, text=True, encoding='utf-8', errors='replace')
check('V13 malformed rows do not turn a VIOLATION into UNPROVEN',
      _p.returncode == 4 and 'VERDICT: UNAIDED VIOLATION' in _p.stdout
      and 'Traceback' not in (_p.stdout + _p.stderr),
      f'exit {_p.returncode}' if _p.returncode == 4
      else f'exit {_p.returncode}: {(_p.stdout + _p.stderr)[-300:]}')
_p = subprocess.run([sys.executable, '-X', 'utf8', '-B',
                     os.path.join(REPO, 'tests', 'stress', 'provenance_audit.py'),
                     '--workdir', wd],
                    capture_output=True, text=True, encoding='utf-8', errors='replace')
check('V13 ...including when the audit picks the board itself (the newest row is malformed)',
      _p.returncode == 4 and 'VERDICT: UNAIDED VIOLATION' in _p.stdout,
      f'exit {_p.returncode}' if _p.returncode == 4
      else f'exit {_p.returncode}: {(_p.stdout + _p.stderr)[-300:]}')
_c, _d = PA.audit(wd, A)
check('...and they are counted, not silently dropped',
      (_d.get('lineage_detail') or {}).get('malformed_rows') == 2, str(_d.get('lineage_detail')))

wd, st = fresh()
A = os.path.join(wd, 'A.kicad_pcb')
lever_write(st, A, [mv(st, X)])
_fx = fp_of(A, X)
_t = os.path.join(outside(), 'f.kicad_pcb')
quiet(write_placed_output, A, _t, [{'reference': X, 'new_x': _fx.x, 'new_y': _fx.y,
                                    'new_rotation': _fx.rotation, 'new_side': 'B'}])
shutil.copyfile(_t, A)
grade('V16 a hand FLIP in place of a claimed part is caught', wd, A, PA.VIOLATION,
      drifted_refs=[X])

wd, st = fresh()
A = os.path.join(wd, 'A.kicad_pcb')
lever_write(st, A, [mv(st, X), mv(st, Y)])
_sx = fp_of(st, X)
_t = os.path.join(outside(), 'r.kicad_pcb')
quiet(write_placed_output, A, _t, [{'reference': X, 'new_x': _sx.x, 'new_y': _sx.y,
                                    'new_rotation': _sx.rotation}])
shutil.copyfile(_t, A)
grade('V17 a hand REVERT of one engine move is caught', wd, A, PA.VIOLATION,
      drifted_refs=[X], lineage='unrecorded')

# The nearest state, not the newest row. Two candidates move the same three
# refs to different places; a hand edit of ONE ref in a copy of the first names
# that ref, where "compare with the newest row" would name all three.
wd, st = fresh()
a, b = os.path.join(wd, 'a.kicad_pcb'), os.path.join(wd, 'b.kicad_pcb')
lever_write(st, a, [mv(st, r) for r in (X, Y, Z)])
lever_write(st, b, [mv(st, r, dx=6.0, dy=4.0) for r in (X, Y, Z)])
Ah = os.path.join(wd, 'a_hand.kicad_pcb')
shutil.copyfile(a, Ah)
hand_edit(Ah, X)
grade('lap: a hand edit in a copy of candidate a names that one part', wd, Ah,
      PA.VIOLATION, drifted_refs=[X])

# A PADLESS footprint on esp_prog, moved by hand after a recorded write.
wd, st = fresh(EP)
A = os.path.join(wd, 'A.kicad_pcb')
lever_write(st, A, [mv(st, 'C3')])
with open(A, encoding='utf-8') as f:
    _atxt = f.read()
_blk = next(bk for bk in iter_footprint_blocks(_atxt) if bk[4] == _padless)
_am = re.search(r'\(at\s+([\d.-]+)', _blk[2])
_nb = _blk[2][:_am.start(1)] + f'{float(_am.group(1)) + 3.0:.6f}' + _blk[2][_am.end(1):]
with open(A, 'w', encoding='utf-8', newline='') as f:
    f.write(_atxt[:_blk[0]] + _nb + _atxt[_blk[1]:])
grade('V16b a padless footprint moved by hand is unclaimed', wd, A, PA.VIOLATION,
      unclaimed_refs=[_padless])

# --- legitimate flows: all CLEAN -----------------------------------------
wd, st = fresh()
A = os.path.join(wd, 'A.kicad_pcb')
lever_write(st, A, [mv(st, X)])
with open(A, encoding='utf-8') as f:
    _atxt = f.read()
_cut = _atxt.rstrip().rfind(')')
with open(A, 'w', encoding='utf-8', newline='') as f:            # routed copper
    f.write(_atxt[:_cut] + '\t(segment (start 1 1) (end 2 2) (width 0.2) '
            '(layer "F.Cu") (net 0))\n' + _atxt[_cut:])
lever_write(A, A, [mv(A, Y)], lever='route.py')               # #666's cap move
grade('C5 routed copper then an in-place cap move is CLEAN', wd, A, PA.CLEAN,
      lineage='verified', drifted_refs=[])

wd, st = fresh()
A = os.path.join(wd, 'A.kicad_pcb')
lever_write(st, A, [mv(st, X)])
stamp_locked(A, [X, Y])
grade('C6 a lock stamp after the write is CLEAN', wd, A, PA.CLEAN, lineage='verified')

wd, st = fresh()
A = os.path.join(wd, 'A.kicad_pcb')
_fx = fp_of(st, X)
lever_write(st, A, [{'reference': X, 'new_x': _fx.x, 'new_y': _fx.y,
                     'new_rotation': _fx.rotation, 'new_side': 'B'}])
grade('C13 a lever flip is CLEAN', wd, A, PA.CLEAN, lineage='verified')

# place_seed's polish shape: the row names OUT.polish, and os.replace puts the
# board at OUT. Linked by arrangement, the path does not matter.
wd, st = fresh()
OUT = os.path.join(wd, 'seeded.kicad_pcb')
lever_write(st, OUT, [mv(st, r) for r in (X, Y, Z)], lever='place_seed.py')
stamp_locked(OUT, [Z])
lever_write(OUT, OUT + '.polish', [mv(OUT, X, dx=1.0), mv(OUT, Y, dx=-1.0)],
            lever='place_seed.py')
os.replace(OUT + '.polish', OUT)
grade('C4 a polish written beside the output and renamed onto it is CLEAN', wd, OUT,
      PA.CLEAN, lineage='verified', drifted_refs=[])

wd, st = fresh()
D2b = os.path.join(wd, 'added.kicad_pcb')
lever_write(st, D2b, [mv(st, X)])
with open(D2b, encoding='utf-8') as f:
    _dtxt = f.read()
_blk = next(bk for bk in iter_footprint_blocks(_dtxt) if bk[4] == Y)
_dup = re.sub(r'"%s"' % re.escape(Y), '"R999"', _blk[2], count=1)
with open(D2b, 'w', encoding='utf-8', newline='') as f:
    f.write(_dtxt[:_blk[1]] + '\n' + _dup + _dtxt[_blk[1]:])
_c, _d = grade('D2 a hand-ADDED part alone stays CLEAN, disclosed', wd, D2b, PA.CLEAN,
               lineage='unrecorded')
check('...and the addition is named', 'R999' in (_d.get('added_refs') or []), str(_d.get('added_refs')))


# --- a ledger with no lineage to walk, and links that cannot be computed ---
def _strip(r):
    if 'redacted' not in r:
        r.pop('parent_pose_sha256', None)
        r.pop('board_pose_sha256', None)
    return r


wd, st, A = s972()
F = os.path.join(wd, 'final.kicad_pcb')
lever_write(A, F, [mv(A, Y)])
rewrite_ledger(wd, _strip)
grade('C15 #972 on a pre-digest ledger is still caught (the any-pose check)', wd, F,
      PA.VIOLATION, drifted_refs=[X], lineage='legacy')

wd, st = fresh()
A, F = os.path.join(wd, 'A.kicad_pcb'), os.path.join(wd, 'final.kicad_pcb')
lever_write(st, A, [mv(st, X)])
lever_write(A, F, [mv(A, Y)])
rewrite_ledger(wd, _strip)
grade('C15 an honest pre-digest ledger keeps its old reading, unverifiable named', wd, F,
      PA.CLEAN, lineage='legacy', unverifiable_claims=[X])


def _breaks_last(value):
    def fn(r, _n=[0]):
        _n[0] += 1
        if _n[0] == 2:
            r['board_pose_sha256'] = value(r.get('board_pose_sha256'))
        return r
    return fn


for tag, value, word in (('U5 an unknown digest scheme', lambda d: 'p9:' + d.split(':', 1)[1], 'scheme'),
                         ('U6 a digest that could not be computed', lambda d: None, 'missing')):
    wd, st = fresh()
    A, F = os.path.join(wd, 'A.kicad_pcb'), os.path.join(wd, 'final.kicad_pcb')
    lever_write(st, A, [mv(st, X)])
    lever_write(A, F, [mv(A, Y)])
    rewrite_ledger(wd, _breaks_last(value))
    _c, _d = grade(f'{tag} is UNPROVEN, never a violation', wd, F, PA.UNPROVEN,
                   lineage='unlinkable')
    check('...and says why', word in (_d.get('reason') or ''), _d.get('reason'))

# An old staging epoch: the regime was re-armed over a DIFFERENT board, and a
# board built from the previous baseline is delivered.
wd, st = fresh()
A = os.path.join(wd, 'A.kicad_pcb')
lever_write(st, A, [mv(st, X)])
lever_write(st, st, [mv(st, Z, dx=7.0)], lever='stage_blind.py')
PV.start_regime(wd, st)
_c, _d = grade('U7 a board from a superseded staging is not CLEAN', wd, A, PA.VIOLATION,
               lineage='broken')
settled('lineage')


# --- the second verifier round's findings ---------------------------------
print('3b. small edits, renames, order, and unreadable inputs')

# Small edits. Every hand edit above moves 9/5 mm, so a drift tolerance of a
# millimetre -- or one that ignored rotation -- passed them all.
wd, st = fresh()
A = os.path.join(wd, 'A.kicad_pcb')
lever_write(st, A, [mv(st, X)])
hand_edit(A, X, dx=0.5, dy=0.0)
grade('a 0.5 mm hand nudge of an engine-moved part is caught', wd, A, PA.VIOLATION,
      drifted_refs=[X])
wd, st = fresh()
A = os.path.join(wd, 'A.kicad_pcb')
lever_write(st, A, [mv(st, X)])
hand_edit(A, X, dx=0.0, dy=0.0, rot=(fp_of(A, X).rotation or 0.0) + 2.0)
grade('a 2 degree hand rotation alone is caught', wd, A, PA.VIOLATION, drifted_refs=[X])


def rename_and_move(board, ref, new_ref, dx=0.0, to=None):
    """Rename `ref` in raw text, optionally moving it by `dx` or TO an exact
    (x, y, rot) -- no writer, no row."""
    with open(board, encoding='utf-8') as fh:
        txt = fh.read()
    blk = next(bk for bk in iter_footprint_blocks(txt) if bk[4] == ref)
    body = re.sub(r'(\(property\s+"Reference"\s+)"%s"' % re.escape(ref),
                  r'\1"%s"' % new_ref, blk[2], count=1)
    if dx:
        am = re.search(r'\(at\s+([\d.-]+)', body)
        body = body[:am.start(1)] + f'{float(am.group(1)) + dx:.6f}' + body[am.end(1):]
    if to is not None:
        am = re.search(r'\(at\s+[\d.-]+\s+[\d.-]+(?:\s+[\d.-]+)?\)', body)
        body = (body[:am.start()] + f'(at {to[0]:.6f} {to[1]:.6f} {to[2]:.6g})'
                + body[am.end():])
    with open(board, 'w', encoding='utf-8', newline='') as fh:
        fh.write(txt[:blk[0]] + body + txt[blk[1]:])


wd, st = fresh()
A = os.path.join(wd, 'A.kicad_pcb')
lever_write(st, A, [mv(st, X)])
rename_and_move(A, X, X + 'X', dx=15.0)
_c, _d = grade('a renamed AND moved part is a pose no lever wrote', wd, A, PA.VIOLATION,
               unclaimed_refs=[X + 'X'])
wd, st = fresh()
A = os.path.join(wd, 'A.kicad_pcb')
lever_write(st, A, [mv(st, X)])
rename_and_move(A, X, X + 'X')
grade('a pure rename (same pose) is UNPROVEN, not an accusation', wd, A, PA.UNPROVEN,
      lineage='unrecorded')
wd, st = fresh()
A = os.path.join(wd, 'A.kicad_pcb')
lever_write(st, A, [mv(st, X)])
with open(A, encoding='utf-8') as fh:
    _txt = fh.read()
_blk = next(bk for bk in iter_footprint_blocks(_txt) if bk[4] == Y)
with open(A, 'w', encoding='utf-8', newline='') as fh:
    fh.write(_txt[:_blk[0]] + _txt[_blk[1]:])
_c, _d = grade('a deleted part is UNPROVEN, and named', wd, A, PA.UNPROVEN)
check('...named', Y in ((_d.get('lineage_detail') or {}).get('missing_refs') or []))

# Ledger order is COMMIT order. An honest two-step chain whose rows landed in
# the other order must still link: the fixpoint repeats until nothing grows.
wd, st = fresh()
A, F = os.path.join(wd, 'A.kicad_pcb'), os.path.join(wd, 'final.kicad_pcb')
lever_write(st, A, [mv(st, X)])
lever_write(A, F, [mv(A, Y)])
_lines = open(os.path.join(wd, PV.LEDGER_NAME), encoding='utf-8').read().splitlines()
with open(os.path.join(wd, PV.LEDGER_NAME), 'w', encoding='utf-8') as fh:
    fh.write('\n'.join(reversed(_lines)) + '\n')
grade('an honest chain whose rows were committed out of order is CLEAN', wd, F,
      PA.CLEAN, lineage='verified')

# The chain is replayed OLDEST first: two writes that each re-move the edited
# part leave the NEWER pose on the board, which is what the replay must expect.
wd, st, A = s972()
B, F = os.path.join(wd, 'B.kicad_pcb'), os.path.join(wd, 'final.kicad_pcb')
lever_write(A, B, [mv(A, X, dx=-4.0, dy=3.0)])
lever_write(B, F, [mv(B, X, dx=2.0, dy=-6.0)])
grade('two writes re-moving the edited part leave it unnameable: UNPROVEN', wd, F,
      PA.UNPROVEN, lineage='broken')

# A write-all lever whose input could not be parsed records EVERY placement as
# moved, at the pose it was handed -- the hand edit included.
wd, st = fresh()
A = os.path.join(wd, 'A.kicad_pcb')
lever_write(st, A, [mv(st, X)])
_fpa = PV.pose_table(st)
hand_edit(A, X)
F = os.path.join(wd, 'final.kicad_pcb')
_all = [{'reference': r, 'new_x': p[0], 'new_y': p[1], 'new_rotation': p[2]}
        for r, p in PV.pose_table(A).items()]
_real = PV.pose_footprints
_Aabs = os.path.normcase(os.path.abspath(A))
PV.pose_footprints = (lambda p: (_ for _ in ()).throw(RuntimeError('unreadable'))
                      if os.path.normcase(os.path.abspath(p)) == _Aabs else _real(p))
try:
    lever_write(A, F, _all)
finally:
    PV.pose_footprints = _real
check('fixture: that row has no parent digest and claims the hand pose',
      rows(wd)[-1].get('parent_pose_sha256') is None and X in rows(wd)[-1]['refs_moved'])
grade('an unreadable input does not let a write-all row vouch for a hand edit', wd, F,
      PA.UNPROVEN, lineage='unlinkable')

# #972 through a write-all lever, plus one pre-digest row: the legacy reading
# must not count a pass-through pose as recorded.
wd, st, A = s972()
F = os.path.join(wd, 'final.kicad_pcb')
_all = [{'reference': r, 'new_x': p[0], 'new_y': p[1], 'new_rotation': p[2]}
        for r, p in PV.pose_table(A).items() if r != Y] + [mv(A, Y)]
lever_write(A, F, _all)
with open(os.path.join(wd, PV.LEDGER_NAME), 'a', encoding='utf-8') as fh:
    fh.write(json.dumps({'schema': 1, 'lever': 'place_seed.py', 'declared': True,
                         'path': os.path.join(wd, 'elsewhere.kicad_pcb'),
                         'refs_moved': [], 'poses_written': {}}) + '\n')
grade('legacy: a write-all pass-through is not a recorded pose', wd, F, PA.VIOLATION,
      drifted_refs=[X], lineage='legacy')
settled('verifier round 2')


# --- the delta verifier's findings ----------------------------------------
print('3c. renames a lever then moved, unreadable rows, and what is relayed')

# A rename the lever then MOVED: the pose is the lever's own, so the renamed
# part is not a pose no lever wrote. The rename itself still leaves a part
# missing, which is UNPROVEN.
wd, st = fresh()
A, F = os.path.join(wd, 'A.kicad_pcb'), os.path.join(wd, 'final.kicad_pcb')
lever_write(st, A, [mv(st, X)])
rename_and_move(A, Y, 'R99')
lever_write(A, F, [mv(A, 'R99', dx=3.0)])
grade('a renamed part a lever then moved is not accused', wd, F, PA.UNPROVEN,
      unclaimed_refs=[])

# A rename-and-move ONTO another expected part's pose is still a pose no lever
# wrote: only the MISSING parts' expected poses can explain an added one.
wd, st = fresh()
A = os.path.join(wd, 'A.kicad_pcb')
lever_write(st, A, [mv(st, X)])
_zp = PV.pose_table(A)[Z]
rename_and_move(A, Y, 'R98', to=_zp[:3])
check('fixture: the renamed part sits EXACTLY on the other part\'s pose',
      not PA._pose_differs(PV.pose_table(A)['R98'], _zp), str(PV.pose_table(A).get('R98')))
_c, _d = grade('a renamed part moved onto ANOTHER part\'s pose is unclaimed', wd, A,
               PA.VIOLATION, unclaimed_refs=['R98'])

# A hand edit AFTER a row whose input could not be parsed: that row's poses
# cannot clear the part, but the delivered pose matches none of them either.
wd, st = fresh()
A, F = os.path.join(wd, 'A.kicad_pcb'), os.path.join(wd, 'final.kicad_pcb')
lever_write(st, A, [mv(st, X), mv(st, Y)])
_all = [{'reference': r, 'new_x': p[0], 'new_y': p[1], 'new_rotation': p[2]}
        for r, p in PV.pose_table(A).items()]
_Aabs = os.path.normcase(os.path.abspath(A))
PV.pose_footprints = (lambda p: (_ for _ in ()).throw(RuntimeError('unreadable'))
                      if os.path.normcase(os.path.abspath(p)) == _Aabs else _real(p))
try:
    lever_write(A, F, _all)
finally:
    PV.pose_footprints = _real
hand_edit(F, Y, dx=20.0, dy=0.0)
grade('a hand edit after an unreadable-input row is still a VIOLATION', wd, F,
      PA.VIOLATION, drifted_refs=[Y], lineage='unlinkable')

# A write naming a ref the board does not carry: the writer warns and skips
# it, so the replay must not invent that part and then report it missing.
wd, st = fresh()
A = os.path.join(wd, 'A.kicad_pcb')
lever_write(st, A, [mv(st, X), {'reference': 'NOPE9', 'new_x': 1.0, 'new_y': 1.0,
                                'new_rotation': 0.0}])
grade('a write naming a ref the board lacks is still CLEAN', wd, A, PA.CLEAN,
      lineage='verified')

# What run_watch relays: the reason line (the only place drifted parts and the
# break are named) and the lineage line, not just VERDICT.
import run_watch as RW                                         # noqa: E402
_out = ('VERDICT: UNAIDED VIOLATION\n  1 pose(s) are NOT where ... (C1).\n'
        '  lineage: broken (compared with x; break at y)\n'
        'JSON_SUMMARY: {"verdict": "UNAIDED VIOLATION"}\n')
_rel = RW._provenance_lines(_out)
check('run_watch relays the verdict, the reason naming the part, and the lineage',
      len(_rel) == 3 and '(C1)' in _rel[1] and _rel[2].startswith('lineage:'), str(_rel))
settled('delta round')


print(f'\n{passed} passed, {failed} failed')
sys.exit(1 if failed else 0)
