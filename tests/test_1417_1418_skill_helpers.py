#!/usr/bin/env python3
"""
#1417 / #1418: the two helpers the placement and routing skills call.

  pdf_text.py       -- read a datasheet PDF through whatever is on the machine
                       (poppler, pypdf, PyMuPDF, macOS PDFKit).
  kicad_cli_path.py -- print kicad-cli's path the way the tools resolve it.

The PDF is written here with the standard library (two pages, one heading on
page 2), so the test needs no fixture. Every backend present on this host is
exercised and must agree on what it finds; one that is absent must REFUSE
with exit 3 and its reason, never pass. At least one backend must have run,
or the test says so and exits 77 rather than reporting a pass it never made.

Run:
    python3 tests/test_1417_1418_skill_helpers.py
"""

import os
import subprocess
import sys
import tempfile

TESTS_DIR = os.path.dirname(os.path.abspath(__file__))
ROOT_DIR = os.path.dirname(TESTS_DIR)
sys.path.insert(0, TESTS_DIR)
sys.path.insert(0, os.path.join(ROOT_DIR, 'py_tools'))

from run_utils import tool

PAGES = ['Features and Pin Configuration', 'Layout Example: keep C1 close']


def write_pdf(path, lines):
    """A minimal valid PDF, one Helvetica line per page."""
    objs = ['<< /Type /Catalog /Pages 2 0 R >>', None,
            '<< /Type /Font /Subtype /Type1 /BaseFont /Helvetica >>']
    kids = []
    for text in lines:
        stream = f'BT /F1 18 Tf 72 720 Td ({text}) Tj ET'
        objs.append(f'<< /Length {len(stream)} >>\nstream\n{stream}\nendstream')
        objs.append('<< /Type /Page /Parent 2 0 R /MediaBox [0 0 612 792] '
                    f'/Resources << /Font << /F1 3 0 R >> >> '
                    f'/Contents {len(objs)} 0 R >>')
        kids.append(f'{len(objs)} 0 R')
    objs[1] = (f'<< /Type /Pages /Kids [{" ".join(kids)}] '
               f'/Count {len(lines)} >>')
    out = b'%PDF-1.4\n'
    offsets = []
    for i, body in enumerate(objs, 1):
        offsets.append(len(out))
        out += f'{i} 0 obj\n{body}\nendobj\n'.encode('latin-1')
    xref = len(out)
    out += f'xref\n0 {len(objs) + 1}\n0000000000 65535 f \n'.encode()
    out += b''.join(f'{o:010d} 00000 n \n'.encode() for o in offsets)
    out += (f'trailer\n<< /Size {len(objs) + 1} /Root 1 0 R >>\n'
            f'startxref\n{xref}\n%%EOF\n').encode()
    with open(path, 'wb') as f:
        f.write(out)


def run(*argv):
    return subprocess.run([sys.executable, *argv], capture_output=True,
                          text=True)


def main():
    fails = []

    def check(name, ok, detail=''):
        print(('  PASS ' if ok else '  FAIL ') + name
              + ('' if ok else f'\n       {detail}'))
        if not ok:
            fails.append(name)

    tmp = tempfile.mkdtemp(prefix='t1417_')
    pdf = os.path.join(tmp, 'ds.pdf')
    write_pdf(pdf, PAGES)
    pdf_text = tool('pdf_text.py')

    ran_text = ran_render = 0
    for backend in ('poppler', 'pypdf', 'pymupdf', 'pdfkit'):
        r = run(pdf_text, pdf, '--find', 'layout example',
                '--backend', backend)
        if r.returncode == 3:
            check(f'{backend}: absent, refuses with its reason',
                  'no backend could do this' in r.stderr, r.stderr)
            continue
        ran_text += 1
        check(f'{backend}: --find names page 2',
              r.returncode == 0 and 'page 2: Layout Example' in r.stdout
              and 'page 1:' not in r.stdout, r.stdout + r.stderr)
        r = run(pdf_text, pdf, '--pages', '1', '--backend', backend)
        check(f'{backend}: --pages 1 prints page 1 only',
              r.returncode == 0 and '=== page 1 ===' in r.stdout
              and 'Features' in r.stdout and 'Layout' not in r.stdout,
              r.stdout + r.stderr)
        r = run(pdf_text, pdf, '--pages', '3', '--backend', backend)
        check(f'{backend}: a page past the end is refused (exit 2)',
              r.returncode == 2 and 'outside 1-2' in r.stderr, r.stderr)

        png = os.path.join(tmp, f'p2_{backend}.png')
        r = run(pdf_text, pdf, '--render', '2', '--out', png,
                '--dpi', '72', '--backend', backend)
        if r.returncode == 3:
            check(f'{backend}: cannot render, refuses with its reason',
                  'no backend could do this' in r.stderr, r.stderr)
            continue
        ran_render += 1
        with open(png, 'rb') as f:
            head = f.read(8) if os.path.isfile(png) else b''
        check(f'{backend}: --render writes a PNG',
              r.returncode == 0 and head == b'\x89PNG\r\n\x1a\n',
              r.stdout + r.stderr)

    r = run(pdf_text, os.path.join(tmp, 'missing.pdf'), '--find', 'x')
    check('a missing file exits 2', r.returncode == 2, r.stderr)

    # kicad_cli_path: stdout is the path and nothing else, or exit 1.
    r = run(tool('kicad_cli_path.py'))
    if r.returncode == 0:
        path = r.stdout.strip()
        check('kicad_cli_path prints one existing path',
              len(r.stdout.splitlines()) == 1 and os.path.isfile(path),
              r.stdout)
    else:
        check('kicad_cli_path exits 1 with a reason when none is found',
              r.returncode == 1 and 'not found' in r.stderr
              and not r.stdout, r.stdout + r.stderr)
    # The not-found branch, whatever this host has installed.
    import kicad_cli_path as kcp
    import kicad_oracle
    real = kicad_oracle.find_kicad_cli
    kicad_oracle.find_kicad_cli = lambda warn=True: None
    try:
        check('kicad_cli_path returns 1 when the resolver finds nothing',
              kcp.main([]) == 1)
    finally:
        kicad_oracle.find_kicad_cli = real

    if fails:
        print(f'\nFAIL ({len(fails)}): ' + '; '.join(fails))
        return 1
    if not ran_text:
        print('\nSKIP: no PDF backend on this host, nothing was exercised')
        return 77
    print(f'\nAll checks passed ({ran_text} text backend(s), '
          f'{ran_render} render backend(s) exercised)')
    return 0


if __name__ == '__main__':
    sys.exit(main())
