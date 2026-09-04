#!/usr/bin/env python3
"""#714 REF: what the flip must REFUSE rather than guess.

A wrong mirror is silent -- `legality.footprint_side` reads `fp.layer[0]` and
nothing cross-checks the pads -- and `write_placed_output` returns True
unconditionally, so a construct the transform quietly passed through is
indistinguishable from success at every call site. That is the same argument
`writer.py`'s #829 guard already makes for raising rather than skipping, and it
is why the default for an unrecognised node here is a REFUSAL.

REFUSING IS CHEAP, MEASURED. Over the 22 tracked boards
(`tests/measure_714_mirror_convention.py` section A, 1349 footprint blocks) the
refusal set touches at most 7 blocks: `primitives` 6, `zone` 1. Four of the
constructs below have NO corpus witness at all -- `chamfer`, `rect_delta`,
`fp_text_box`, `group` -- which is the whole argument for refusing them: no
corpus gate can validate a rule for a construct the corpus does not contain, so
any rule for it is an unfalsifiable guess. Those four are exercised here by
INJECTING them into a real board, which is the only way to test a refusal for
something nothing ships.

A REFUSAL IS ONLY A GUARD IF IT REFUSES FOR ITS OWN REASON. An `ImportError`,
an `AttributeError` or a `KeyError` also stops the write, and reads identical
from outside. So every case below asserts the exception TYPE and requires the
message to name both the ref and the construct; anything else is reported as a
BROKEN TEST rather than as a guard that held (`tests/run_utils.py`'s `check`
makes the same distinction for subprocesses).

    python3 tests/test_714_refusals.py
"""
from __future__ import annotations

import os
import re
import shutil
import sys
import tempfile

TESTS = os.path.dirname(os.path.abspath(__file__))
REPO = os.path.dirname(TESTS)
sys.path.insert(0, os.path.join(REPO, 'py_router'))
sys.path.insert(0, os.path.join(REPO, 'py_placer'))
sys.path.insert(0, TESTS)

RUN_ALL_TIMEOUT = 300

# Exceptions that mean the TEST broke, not that the guard held.
_ACCIDENTS = (ImportError, AttributeError, KeyError, IndexError, NameError,
              TypeError, FileNotFoundError)


def _inject(src, dst, ref, extra=None, pad_layers=None, sub=None):
    """Copy `src` to `dst`, editing only the footprint block named `ref`.

    `extra`       an s-expression appended as a new top-level child
    `pad_layers`  replace the first pad's `(layers ...)` with this text
    `sub`         (pattern, replacement) applied once inside the block
    """
    from kicad_parser import iter_footprint_blocks
    text = open(src, encoding='utf-8').read()
    out = None
    for s, e, fp_text, _raw, key in iter_footprint_blocks(text):
        if key != ref:
            continue
        body = fp_text
        if sub is not None:
            body = re.sub(sub[0], sub[1], body, count=1)
        if pad_layers is not None:
            body = re.sub(r'\(layers[^)]*\)', pad_layers, body, count=1)
        if extra is not None:
            close = body.rindex(')')
            body = body[:close] + '\n\t\t' + extra + '\n\t' + body[close:]
        out = text[:s] + body + text[e:]
        break
    if out is None:
        raise AssertionError(f"{ref} not found in {src} -- fixture stale")
    with open(dst, 'w', encoding='utf-8') as fh:
        fh.write(out)
    return dst


def _expect_refusal(label, ref, needles, fn):
    """Run `fn`; require a refusal that names `ref` and every needle."""
    from placement.writer import write_placed_output  # noqa: F401
    try:
        from placement.writer import SideFlipUnsupported
    except ImportError:
        return (label, 'NO REFUSAL TYPE',
                'placement.writer defines no SideFlipUnsupported')
    try:
        fn()
    except SideFlipUnsupported as exc:
        msg = str(exc)
        missing = [n for n in ([ref] + list(needles)) if n.lower() not in msg.lower()]
        if missing:
            return (label, 'UNNAMED',
                    f"refused, but the message names neither {missing}: {msg!r}")
        return (label, 'OK', msg)
    except _ACCIDENTS as exc:
        return (label, 'BROKEN TEST',
                f"{type(exc).__name__}: {exc} -- this is an accident, not a guard")
    except Exception as exc:                                     # noqa: BLE001
        return (label, 'WRONG TYPE',
                f"{type(exc).__name__}: {exc}")
    return (label, 'NOT REFUSED', 'the write succeeded')


def main():
    from kicad_parser import parse_kicad_pcb
    from placement.writer import write_placed_output

    tmp = tempfile.mkdtemp(prefix='krt714ref')
    results = []
    try:
        def flip(path, ref, side=None):
            fp = parse_kicad_pcb(path).footprints[ref]
            if side is None:
                side = 'F' if (fp.layer or 'F').startswith('B') else 'B'
            out = os.path.join(tmp, 'out.kicad_pcb')
            return write_placed_output(path, out, [{
                'reference': ref, 'new_x': round(fp.x, 6),
                'new_y': round(fp.y, 6), 'new_rotation': fp.rotation,
                'new_side': side}])

        rp = os.path.join(REPO, 'kicad_files',
                          'rp2350_fpga_eensy_prePlane.kicad_pcb')
        oc = os.path.join(REPO, 'kicad_files', 'orangecrab_ext_pll.kicad_pcb')
        tg = os.path.join(REPO, 'kicad_files', 'tigard.kicad_pcb')

        # --- the two constructs the corpus DOES carry
        results.append(_expect_refusal(
            'footprint-internal (zone) [rp2350 J2, the only corpus witness]',
            'J2', ['zone'], lambda: flip(rp, 'J2')))
        results.append(_expect_refusal(
            'custom pad (primitives) [orangecrab U9]',
            'U9', ['primitives'], lambda: flip(oc, 'U9')))

        # --- the four with NO corpus witness, injected
        p = _inject(tg, os.path.join(tmp, 'textbox.kicad_pcb'), 'J7',
                    extra='(fp_text_box "x" (start 0 0) (end 1 1) '
                          '(layer "F.SilkS"))')
        results.append(_expect_refusal('unknown node kind (fp_text_box)',
                                       'J7', ['fp_text_box'],
                                       lambda: flip(p, 'J7')))
        p = _inject(tg, os.path.join(tmp, 'chamfer.kicad_pcb'), 'J7',
                    sub=(r'\(roundrect_rratio [^)]*\)',
                         '(chamfer top_left bottom_left)'))
        results.append(_expect_refusal('(chamfer ...) corner names',
                                       'J7', ['chamfer'],
                                       lambda: flip(p, 'J7')))
        p = _inject(tg, os.path.join(tmp, 'fandb.kicad_pcb'), 'J7',
                    pad_layers='(layers "F&B.Cu" "F.Mask" "F.Paste")')
        results.append(_expect_refusal('F&B.Cu in a pad (layers)',
                                       'J7', ['F&B'], lambda: flip(p, 'J7')))
        p = _inject(tg, os.path.join(tmp, 'inner.kicad_pcb'), 'J7',
                    pad_layers='(layers "In1.Cu" "F.Mask" "F.Paste")')
        results.append(_expect_refusal('In<n>.Cu in a pad (layers)',
                                       'J7', ['In1.Cu'], lambda: flip(p, 'J7')))

        # --- a coordinate literal outside the measured grammar. 148417 tokens
        # on the corpus, 0 outside `-?\d+(\.\d+)?`; a sign toggle is an exact
        # involution only over that shape, so anything else must refuse.
        p = _inject(tg, os.path.join(tmp, 'expo.kicad_pcb'), 'J7',
                    sub=(r'\(at -1\.5 -2\b', '(at -1.5 -2e0'))
        results.append(_expect_refusal('coordinate outside the literal grammar',
                                       'J7', ['2e0'], lambda: flip(p, 'J7')))

        # --- a bad side vocabulary is a caller error, caught at the boundary
        try:
            flip(tg, 'J7', side='B.Cu')
            results.append(('new_side="B.Cu" (the mistake a caller makes)',
                            'NOT REFUSED', 'accepted a layer name as a side'))
        except ValueError as exc:
            results.append(('new_side="B.Cu" (the mistake a caller makes)',
                            'OK', str(exc)))
        except _ACCIDENTS as exc:
            results.append(('new_side="B.Cu" (the mistake a caller makes)',
                            'BROKEN TEST', f"{type(exc).__name__}: {exc}"))
        except Exception as exc:                                 # noqa: BLE001
            results.append(('new_side="B.Cu" (the mistake a caller makes)',
                            'WRONG TYPE', f"{type(exc).__name__}: {exc}"))
    finally:
        shutil.rmtree(tmp, ignore_errors=True)

    bad = [r for r in results if r[1] != 'OK']
    for label, status, detail in results:
        print(f"  {status:<16} {label}")
        if status != 'OK':
            print(f"                   {detail}")
    if bad:
        print(f"FAIL: {len(bad)} of {len(results)} refusals did not hold")
        return 1
    print(f"PASS: {len(results)} refusals hold and name their construct")
    return 0


if __name__ == '__main__':
    sys.exit(main())
