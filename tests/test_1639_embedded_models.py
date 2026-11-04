#!/usr/bin/env python3
"""#1639: a 3D model embedded in the board file is not missing, and no
recorded answer replaces it.

KiCad 10 can keep a footprint's model inside the board: the footprint names
`kicad-embed://NAME` and the file is in an `(embedded_files ...)` block, the
board's or the footprint's own. model_export used to resolve that reference
as a path under the board's folder, which is never a file, so
`find_models missing` listed every such model, the movie's `record:` line
said the library lacked models it had just drawn, and a recorded
`find_models` answer rewrote the board's own reference.

Checks, on the parts of tests/fixtures/714/flip_witnesses.kicad_pcb whose
models are embedded (WEMB1-3, one of them embedded as `(type other)`):
  1. stage_models with no library: the three are counted `embedded`, none is
     missing, and each reference is staged unchanged.
  2. A recorded answer for one of them does not replace it.
  3. A name in the footprint's own list alone, or in the board's alone, is
     embedded either way.
  4. A `kicad-embed://` name the board does not carry (the fixture's WPT1)
     is still missing with no library; a library file of its name still
     stands in.
  5. cover_facts reports the embedded models resolved.
  6. With kicad-cli: `find_models missing` lists none of them, and
     models.export draws all three, noting them embedded.

    python3 tests/test_1639_embedded_models.py
"""
import contextlib
import io
import os
import re
import shutil
import sys
import tempfile

TESTS_DIR = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.dirname(TESTS_DIR)
for sub in ('py_router', 'py_placer', 'py_tools', 'movie3d'):
    sys.path.insert(0, os.path.join(ROOT, sub))

from kicad_parser import iter_footprint_blocks                 # noqa: E402
import model_export                                            # noqa: E402

FIXTURE = os.path.join(ROOT, 'tests', 'fixtures', '714', 'flip_witnesses.kicad_pcb')
EMB = {'WEMB1', 'WEMB2', 'WEMB3'}
REFS = ['kicad-embed://2286211-3.step',
        'kicad-embed://Download_STP_WL-SBSW-1204 (rev1).stp',
        'kicad-embed://hc-49us.stp']
failures = []


def check(label, ok, detail=''):
    print(f"  [{'ok' if ok else 'FAIL'}] {label}{(': ' + detail) if detail else ''}")
    if not ok:
        failures.append(label)


def only(text, keep):
    """TEXT with every footprint block but KEEP's dropped."""
    out, last = [], 0
    for start, end, _fp, raw, key in iter_footprint_blocks(text):
        out.append(text[last:start])
        if (raw or key) in keep:
            out.append(text[start:end])
        last = end
    out.append(text[last:])
    return ''.join(out)


def drop_blocks(text, head):
    """TEXT without its paren-balanced blocks opening with HEAD."""
    out, i = [], 0
    while True:
        j = text.find(head, i)
        if j < 0:
            return ''.join(out) + text[i:]
        depth, k = 0, j
        while True:
            depth += (text[k] == '(') - (text[k] == ')')
            k += 1
            if depth == 0:
                break
        out.append(text[i:j])
        i = k


def footprint_level_only(text):
    """Drop the board-level embedded_files, keep the footprints' lists."""
    i = text.rfind('\n\t(embedded_files')      # the one top-level block
    assert i > 0
    return text[:i + 1] + drop_blocks(text[i + 1:], '(embedded_files')


def board_level_only(text):
    """Drop every footprint's embedded_files, keep the board's."""
    out, last = [], 0
    for start, end, fp, _raw, _key in iter_footprint_blocks(text):
        out.append(text[last:start] + drop_blocks(fp, '(embedded_files'))
        last = end
    out.append(text[last:])
    return ''.join(out)


def staged_refs(text):
    return re.findall(r'\(model\s+"([^"]+)"', text)


def main():
    with open(FIXTURE, encoding='utf-8') as f:
        full = f.read()
    text = only(full, EMB)
    tmp = tempfile.mkdtemp(prefix='t1639_')
    nolib = {'KIPRJMOD': tmp}
    try:
        print("1. stage_models, no library")
        staged, counts, missing, _f = model_export.stage_models(text, nolib, tmp)
        check("three counted embedded, none missing",
              counts['embedded'] == 3 and missing == [], f"{counts} {missing}")
        check("each reference staged unchanged", staged_refs(staged) == REFS,
              repr(staged_refs(staged)))

        print("2. a recorded answer does not replace an embedded model")
        stand_in = os.path.join(tmp, 'stand_in.step')
        with open(stand_in, 'w') as f:
            f.write('ISO-10303-21;\n')
        found = {'hc-49us': {'status': 'found', 'file': stand_in, 'match': 'equivalent'}}
        staged, counts, missing, found_refs = model_export.stage_models(
            text, nolib, tmp, found=found)
        check("hc-49us keeps its kicad-embed:// reference, nothing drawn from the answer",
              'kicad-embed://hc-49us.stp' in staged_refs(staged)
              and counts['found'] == 0 and found_refs == [], f"{counts} {found_refs}")

        print("3. the footprint's list or the board's, either is enough")
        for label, t in (("footprint's list only", footprint_level_only(text)),
                         ("board's list only", board_level_only(text))):
            _s, counts, missing, _f = model_export.stage_models(t, nolib, tmp)
            check(label, counts['embedded'] == 3 and missing == [], f"{counts} {missing}")
        neither = board_level_only(footprint_level_only(text))
        _s, counts, missing, _f = model_export.stage_models(neither, nolib, tmp)
        check("with no list at all the three are missing (the control)",
              counts['embedded'] == 0 and sorted(missing) == sorted(REFS), repr(missing))

        print("4. a kicad-embed:// name the board does not carry")
        wpt = only(full, {'WPT1'})
        _s, counts, missing, _f = model_export.stage_models(wpt, nolib, tmp)
        check("missing with no library", missing == ['kicad-embed://LED_D5.0mm.step'],
              repr(missing))
        lib = os.path.join(tmp, 'lib')
        os.makedirs(lib)
        shutil.copyfile(stand_in, os.path.join(lib, 'LED_D5.0mm.step'))
        staged, counts, missing, _f = model_export.stage_models(
            wpt, {'KIPRJMOD': tmp, 'KICAD10_3DMODEL_DIR': lib}, tmp)
        check("a library file of its name stands in",
              counts['named'] == 1 and missing == []
              and staged_refs(staged) == [os.path.join(lib, 'LED_D5.0mm.step').replace('\\', '/')],
              f"{counts} {staged_refs(staged)}")

        board = os.path.join(tmp, 'embedded.kicad_pcb')
        with open(board, 'w', encoding='utf-8') as f:
            f.write(text)

        print("5. cover_facts")
        import cover_facts
        rows = cover_facts._models(text, board, EMB)
        check("every embedded model resolved",
              sorted(r['path'] for v in rows.values() for r in v if r['resolved']) == sorted(REFS),
              repr(rows))

        print("6. with kicad-cli")
        cli, why = model_export.resolve_cli()
        if not cli:
            print(f"  [skip] {why}")
        else:
            import find_models
            import models
            with contextlib.redirect_stdout(io.StringIO()), \
                    contextlib.redirect_stderr(io.StringIO()):
                rep = find_models.missing_report(board)
                info, why = models.export(text, tmp, sorted(EMB),
                                          cache=os.path.join(tmp, 'cache'))
            check("find_models missing lists none of them",
                  [m['key'] for m in rep['missing']] == [], repr(rep['missing']))
            check("kicad-cli draws all three, noted embedded",
                  info is not None and sorted(info['matched']) == sorted(EMB)
                  and info['missing'] == [] and '3 embedded in the board' in info['note'],
                  repr(info and {k: info[k] for k in ('matched', 'missing', 'note')}) or why)
    finally:
        shutil.rmtree(tmp, ignore_errors=True)
    if failures:
        print(f"\nFAIL: {len(failures)} check(s): {failures}")
        return 1
    print("\nPASS")
    return 0


if __name__ == '__main__':
    sys.exit(main())
