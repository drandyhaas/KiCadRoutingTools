#!/usr/bin/env python3
"""#1098 over IPC: the kipy builder fills Footprint.attrs / has_model / dnp / locked.

Both of main's parse paths fill a footprint's ``(attr ...)`` flags and whether
it declares a 3D model (#1098). The kipy builder filled neither, and it read
DNP and the lock through pcbnew's ``IsDNP()`` / ``IsLocked()`` -- accessors a
kipy footprint does not have -- so on the IPC front every part read as
populated and unlocked. ``kipy_footprint_attrs`` is the read, a function so
this test can reach it without a running KiCad; the builder takes ``dnp`` from
its tokens and the lock from ``kipy_locked``.

Uses REAL kipy protos when kipy imports (``pynng``, which only kipy's client
needs, is stubbed if absent), else stand-ins shaped like kipy 0.7.1's wrapper:
four flags as properties, the courtyard and soldermask-bridge flags on the
proto only. The arm that ran is printed.

    python3 -X utf8 tests/test_1098_kipy_footprint_attrs.py
"""
import os
import sys
import types

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, os.path.join(ROOT, 'py_router'))
from kicad_parser import (FOOTPRINT_ATTR_TOKENS, kipy_footprint_attrs,  # noqa: E402
                          kipy_locked)

FAILS = []


def check(name, cond, detail=''):
    print(f"  {'ok  ' if cond else 'FAIL'} {name}" + (f"  ({detail})" if detail and not cond else ''))
    if not cond:
        FAILS.append(name)


def _real():
    """(full, control) FootprintInstances from kipy's own protos, or None."""
    try:
        import pynng  # noqa: F401
    except ImportError:
        sys.modules['pynng'] = types.ModuleType('pynng')
    try:
        from kipy.board_types import FootprintInstance
        from kipy.proto.board import board_types_pb2 as b
        from kipy.proto.common.types import LockedState
        from google.protobuf import any_pb2
    except Exception as e:                                       # noqa: BLE001
        print(f"  (kipy not importable: {type(e).__name__}: {e} -- stand-in arm)")
        return None
    p = b.FootprintInstance()
    a = p.attributes
    a.not_in_schematic = True
    a.exclude_from_bill_of_materials = True
    a.exclude_from_position_files = True
    a.do_not_populate = True
    a.exempt_from_courtyard_requirement = True
    a.allow_soldermask_bridges = True
    a.mounting_style = b.FMS_THROUGH_HOLE
    p.locked = LockedState.LS_LOCKED
    m = any_pb2.Any()
    m.Pack(b.Footprint3DModel(filename='part.step'))
    p.definition.items.append(m)
    q = b.FootprintInstance()
    q.attributes.mounting_style = b.FMS_SMD
    return FootprintInstance(p), FootprintInstance(q)


class _Attrs:
    """kipy 0.7.1's FootprintAttributes shape: four properties, all on .proto."""
    def __init__(self, **kw):
        self.proto = types.SimpleNamespace(**kw)
        for f in ('not_in_schematic', 'exclude_from_bill_of_materials',
                  'exclude_from_position_files', 'do_not_populate',
                  'mounting_style'):
            setattr(self, f, kw.get(f, 0 if f == 'mounting_style' else False))


def _stand_in():
    full = types.SimpleNamespace(
        attributes=_Attrs(not_in_schematic=True, exclude_from_bill_of_materials=True,
                          exclude_from_position_files=True, do_not_populate=True,
                          exempt_from_courtyard_requirement=True,
                          allow_soldermask_bridges=True, mounting_style=1),
        definition=types.SimpleNamespace(models=[object()]), locked=True)
    ctrl = types.SimpleNamespace(
        attributes=_Attrs(exempt_from_courtyard_requirement=False,
                          allow_soldermask_bridges=False, mounting_style=2),
        definition=types.SimpleNamespace(models=[]), locked=False)
    return full, ctrl


def main():
    pair = _real()
    arm = 'real kipy protos' if pair else 'kipy-shaped stand-ins'
    full, ctrl = pair or _stand_in()
    print(f"[{arm}]")
    attrs, model = kipy_footprint_attrs(full)
    want = tuple(sorted(set(FOOTPRINT_ATTR_TOKENS) - {'smd'}))
    check('every flag set reads, spelled as FOOTPRINT_ATTR_TOKENS', attrs == want,
          f"{attrs} != {want}")
    check('a declared 3D model reads', model is True)
    check('the lock reads (kipy_locked)', kipy_locked(full) is True)
    attrs, model = kipy_footprint_attrs(ctrl)
    check('control: an SMD part with nothing set reads only smd', attrs == ('smd',), attrs)
    check('control: no model', model is False)
    check('control: unlocked', kipy_locked(ctrl) is False)
    bare = types.SimpleNamespace()
    check('a footprint kipy exposes no attributes on reads empty, no raise',
          kipy_footprint_attrs(bare) == ((), False))
    print(f"\n{'PASS' if not FAILS else 'FAIL'}: {7 - len(FAILS)}/7 checks ({arm})")
    return 1 if FAILS else 0


if __name__ == '__main__':
    sys.exit(main())
