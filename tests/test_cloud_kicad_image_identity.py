"""The cloud KiCad image: one default, and part of every KiCad wave's arm name.

cloud_replay_sets.py chooses the image and hands it to modal_app.py; both
modal_app.py and awx/modal_whole.py keep a standalone default of their own, and
the three are asserted equal here. A wave's arm name must change when the image
does, because the results volume resumes by arm name.
"""

import os
import re
import sys

RUN_ALL_FAST_OK = True
RUN_ALL_TIMEOUT = 60

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, os.path.join(ROOT, 'tests', 'stress'))

import cloud_replay_sets as crs                                 # noqa: E402

FAILURES = []


def report(name, ok, detail=''):
    print(('  PASS  ' if ok else '  FAIL  ') + name
          + (('  -- ' + detail) if detail else ''))
    if not ok:
        FAILURES.append(name)


def standalone_default(relpath, pattern):
    text = open(os.path.join(ROOT, relpath), encoding='utf-8').read()
    m = re.search(pattern, text, re.MULTILINE)
    return m.group(1) if m else None


def main():
    default = crs.DEFAULT_KICAD_IMAGE
    for rel, pat in (
            ('tests/stress/modal_sweep/modal_app.py',
             r'^KICAD_IMAGE = os\.environ\.get\("KICAD_SWEEP_KICAD_IMAGE", "([^"]+)"\)'),
            ('awx/modal_whole.py', r'^KICAD_IMAGE = "([^"]+)"')):
        got = standalone_default(rel, pat)
        report(f'{rel} default matches the driver', got == default,
               f'{got!r} vs {default!r}')

    report('default image keeps the plain -kc suffix',
           crs.kicad_label_suffix(default) == '-kc')
    other = crs.kicad_label_suffix('kicad/kicad:10.0.0')
    report('another tag gets its own suffix', other == '-kc-10.0.0', other)
    foreign = crs.kicad_label_suffix('ghcr.io/me/kicad:10.0.0')
    report('another registry differs from the same tag on Docker Hub',
           foreign not in ('-kc', other), foreign)

    for label, image, want in (
            ('x', default, 'x-kc'),
            ('x-kc', default, 'x-kc'),
            ('x', 'kicad/kicad:10.0.0', 'x-kc-10.0.0'),
            ('x-kc', 'kicad/kicad:10.0.0', 'x-kc-10.0.0'),
            ('x-kc-10.0.0', 'kicad/kicad:10.0.0', 'x-kc-10.0.0')):
        got = crs.kicad_label(label, image)
        report(f'label {label!r} on {image} -> {want!r}', got == want, got)

    print('\n%s' % ('FAILURES: ' + ', '.join(FAILURES) if FAILURES else 'all pass'))
    return 1 if FAILURES else 0


if __name__ == '__main__':
    sys.exit(main())
