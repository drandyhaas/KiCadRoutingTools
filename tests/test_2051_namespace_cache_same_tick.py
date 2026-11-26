#!/usr/bin/env python3
"""#2051: a namespace rewritten in place inside one timestamp tick reads back.

tool_records caches each file's namespace by its (mtime, size, inode) stamp.
Two same-size writes inside one mtime tick (NTFS's ~16 ms clock, FAT's 2 s)
leave that stamp unchanged, and an in-place rewrite keeps the inode, so the
second read used to return the first write's namespace. The tick is forced
here by pinning mtime back with os.utime, so the test does not depend on the
filesystem's clock resolution.

A stamp older than the racy window still vouches on its own, so a settled
file is not re-hashed on every read.

    python3 tests/test_2051_namespace_cache_same_tick.py
"""
import json
import os
import shutil
import sys
import tempfile
import time

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, os.path.join(ROOT, 'py_router'))
import tool_records as tr  # noqa: E402

FAILS = []


def expect(cond, what):
    print(f"  [{'ok' if cond else 'FAIL'}] {what}")
    if not cond:
        FAILS.append(what)


def write_pro(path, value, mtime_ns):
    with open(path, 'w', encoding='utf-8') as f:     # in place: same inode
        json.dump({tr.NAMESPACE: {'k': value}}, f)
    os.utime(path, ns=(mtime_ns, mtime_ns))


def write_pcb(path, value, mtime_ns):
    with open(path, 'w', encoding='utf-8') as f:
        f.write('(kicad_pcb\n\t' + tr.board_property_sexpr({'k': value}) + '\n)\n')
    os.utime(path, ns=(mtime_ns, mtime_ns))


def main():
    tmp = tempfile.mkdtemp(prefix='t2051_')
    try:
        pro = os.path.join(tmp, 'x.kicad_pro')
        pcb = os.path.join(tmp, 'x.kicad_pcb')

        print('same tick, project file:')
        tick = time.time_ns()
        stale = 0
        for i in range(50):
            for v in (1, 2):
                write_pro(pro, v, tick)
                stale += tr.project_namespace(pro).get('k') != v
        expect(stale == 0, f"every same-tick rewrite reads back ({stale} stale of 100)")

        print('same tick, board file:')
        stale = 0
        for i in range(50):
            for v in (1, 2):
                write_pcb(pcb, v, tick)
                ns = tr.board_namespace(pcb) or {}
                stale += ns.get('k') != v
        expect(stale == 0, f"every same-tick rewrite reads back ({stale} stale of 100)")

        print('a settled stamp still vouches on its own:')
        old = time.time_ns() - 10 * tr._RACY_NS
        write_pro(pro, 3, old)
        expect(tr.project_namespace(pro).get('k') == 3, "first read of a settled file")
        expect(tr._PRO_CACHE[pro][2] is None, "a settled entry keeps no digest")

        print('an entry cached young is trusted once its stamp settles:')
        young = time.time_ns()
        write_pro(pro, 4, young)
        expect(tr.project_namespace(pro).get('k') == 4, "read while young")
        expect(tr._PRO_CACHE[pro][2] is not None, "a young entry keeps its digest")
        saved = tr._RACY_NS
        tr._RACY_NS = 0            # the window has passed
        try:
            expect(tr.project_namespace(pro).get('k') == 4, "verified read after the window")
            expect(tr._PRO_CACHE[pro][2] is None, "the verified entry drops its digest")
        finally:
            tr._RACY_NS = saved
    finally:
        shutil.rmtree(tmp, ignore_errors=True)

    if FAILS:
        print(f"FAILED: {len(FAILS)}")
        return 1
    print('ALL PASS')
    return 0


if __name__ == '__main__':
    sys.exit(main())
