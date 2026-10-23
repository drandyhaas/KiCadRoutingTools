#!/usr/bin/env python3
"""Whole-board text is not kept past the call that read it.

`kicad_parser`'s three text-keyed memos (`_mask_pad_primitives`,
`_footprint_blocks_by_key`, `_footprint_edge_points_by_ref`) let one parse
ask each whole-file question once. Kept afterwards they hold the board's text
and a masked copy -- 216 MB for ddr4_rdimm_tester's 54 MB file, twice that
with `maxsize=2` -- so `parse_kicad_pcb` releases them as it returns, and so
does the writer's own block scan (`kicad_writer._functional_footprint_spans`).

Within one parse the work is SHARED (`kicad_parser._PARSE_MEMO`): the
footprint block ends, the masked copy's blocks (they are the original's), and
each footprint's shape spans, found once for every extractor that walks them.
A parse with the memo must equal one without it, field for field, on boards
whose custom pads carry `(primitives ...)` (the masked copy differs there),
and the memo must be gone when the parse returns.

    python3 tests/test_parser_text_caches.py
"""
import os
import sys

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
for _d in ('py_router', 'py_placer', 'py_tools'):
    sys.path.insert(0, os.path.join(ROOT, _d))

import kicad_parser as K  # noqa: E402
import kicad_writer  # noqa: E402

BOARD = os.path.join(ROOT, 'kicad_files', 'esp_prog.kicad_pcb')
FAILS = []
MEMOS = (K._mask_pad_primitives, K._footprint_blocks_by_key_cached,
         K._footprint_edge_points_by_ref_cached)


def check(ok, what):
    print(f"  {'PASS' if ok else 'FAIL'}  {what}")
    if not ok:
        FAILS.append(what)


def held():
    return [m.cache_info().currsize for m in MEMOS]


def main():
    # `cache_clear` resets the hit counts too, so count the scans themselves:
    # within one parse the block scan is asked for more often than it runs.
    asked, ran = [0], [0]
    wrapper, scan = K._footprint_blocks_by_key, K._footprint_blocks_by_key_uncached

    def counting_wrapper(content):
        asked[0] += 1
        return wrapper(content)

    def counting_scan(content):
        ran[0] += 1
        return scan(content)
    K._footprint_blocks_by_key = counting_wrapper
    K._footprint_blocks_by_key_uncached = counting_scan
    try:
        pcb = K.parse_kicad_pcb(BOARD)
    finally:
        K._footprint_blocks_by_key = wrapper
        K._footprint_blocks_by_key_uncached = scan
    check(bool(pcb.footprints), "the board parsed")
    check(asked[0] >= 2 and ran[0] < asked[0],
          f"the memo still serves repeat questions within one parse "
          f"(asked {asked[0]}, scanned {ran[0]})")
    check(held() == [0, 0, 0], f"no board text is held after a parse {held()}")
    text = K.read_board_text(BOARD)
    kicad_writer._functional_footprint_spans(text)
    check(held() == [0, 0, 0], f"none after the writer's block scan {held()}")

    check(K._PARSE_MEMO is None, "the per-parse memo is gone after the parse")
    import hashlib
    def fields(pcb):
        return {k: hashlib.sha1(repr(v).encode()).hexdigest()
                for k, v in sorted(vars(pcb).items())
                if not (callable(v) and not isinstance(v, (list, dict)))}
    shared = {}
    for name in ('orangecrab_ext_pll.kicad_pcb', 'rp2350_fpga_eensy_prePlane.kicad_pcb',
                 'tigard.kicad_pcb', 'glasgow_revC.kicad_pcb'):
        path = os.path.join(ROOT, 'kicad_files', name)
        with_memo = fields(K.parse_kicad_pcb(path))
        real = K._memo_for
        K._memo_for = lambda content: None
        try:
            without = fields(K.parse_kicad_pcb(path))
        finally:
            K._memo_for = real
        differ = sorted(k for k in with_memo if with_memo[k] != without.get(k))
        check(not differ, f"{name}: the shared parse equals the unshared one "
                          f"(differ: {differ})")
        counts = []
        for share in (True, False):
            hits = [0]
            real_spans = K._footprint_shape_spans
            def counting(fp_text, tags=K._FP_SHAPE_TAGS):
                hits[0] += 1
                return real_spans(fp_text, tags)
            K._footprint_shape_spans = counting
            if not share:
                K._memo_for = lambda content: None
            try:
                K.parse_kicad_pcb(path)
            finally:
                K._footprint_shape_spans = real_spans
                K._memo_for = real
            counts.append(hits[0])
        shared[name] = tuple(counts)
        check(counts[0] < counts[1], f"{name}: footprint shapes are scanned "
                                     f"once per parse ({counts[0]} scans "
                                     f"shared, {counts[1]} unshared)")
    print(f"  footprint shape scans per parse (shared, unshared): {shared}")

    if FAILS:
        print(f"FAIL: {len(FAILS)} check(s)")
        return 1
    print("PASS")
    return 0


if __name__ == '__main__':
    sys.exit(main())
