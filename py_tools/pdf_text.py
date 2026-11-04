#!/usr/bin/env python3
"""Read a datasheet PDF without poppler: find pages, print their text, render one (#1417).

The placement skill reads layout guidelines and the layout figure out of a
manufacturer's PDF. Its recipe needed poppler (`pdftotext`, `pdftoppm`) or a
harness whose Read tool can split pages -- which itself shells out to
poppler -- and a machine without it had no way in: 14 of 20 boards in one
campaign lost 4-10 minutes each to improvised PDF readers.

This tries, in order, whatever is already on the machine, and needs nothing
installed on macOS:

  poppler   `pdftotext` / `pdftoppm` on PATH
  pypdf     the `pypdf` module (text only)
  pymupdf   the `fitz` module
  pdfkit    macOS's own PDFKit, through `osascript -l JavaScript`

Usage:
    python3 py_tools/pdf_text.py DS.pdf --find "Layout Example" "Typical Application"
    python3 py_tools/pdf_text.py DS.pdf --pages 31-33
    python3 py_tools/pdf_text.py DS.pdf --render 32 --out wk/run/fig32.png [--dpi 200]

`--find` prints `page N: <line>` for every line matching any pattern
(case-insensitive regular expressions); `--pages` prints each page's text
under a `=== page N ===` header; `--render` writes page N as a PNG, which the
Read tool opens as an image. Pages are numbered from 1, as a PDF viewer shows
them. The first line on stderr names the backend used.

Exit codes: 0 done (including a `--find` with no match, which says so),
2 bad arguments or an unreadable PDF, 3 no backend on this machine can do it
(the message names what to install).
"""

#: #937 registry: which door(s) show this tool, and whether it changes
#: the board. Read by krt_registry.py -- by AST, never imported.
KRT_TOOL = {'scope': ['placement', 'combined'], 'kind': 'utility'}

import argparse
import json
import os
import re
import shutil
import subprocess
import sys

BACKENDS = ('poppler', 'pypdf', 'pymupdf', 'pdfkit')


class NoBackend(Exception):
    """This backend is not available on this machine."""


# --- text: each returns a list of page strings, page 1 first ---------------

def _text_poppler(pdf):
    exe = shutil.which('pdftotext')
    if not exe:
        raise NoBackend('pdftotext not on PATH')
    r = subprocess.run([exe, '-layout', '-enc', 'UTF-8', pdf, '-'],
                       capture_output=True)
    if r.returncode != 0:
        raise ValueError(r.stderr.decode('utf-8', 'replace').strip()
                         or f'pdftotext exited {r.returncode}')
    pages = r.stdout.decode('utf-8', 'replace').split('\f')
    # pdftotext ends the last page with a form feed too.
    if pages and not pages[-1].strip():
        pages.pop()
    return pages


def _text_pypdf(pdf):
    try:
        from pypdf import PdfReader
    except ImportError:
        raise NoBackend('pypdf not installed')
    return [(p.extract_text() or '') for p in PdfReader(pdf).pages]


def _text_pymupdf(pdf):
    try:
        import fitz
    except ImportError:
        raise NoBackend('PyMuPDF (fitz) not installed')
    with fitz.open(pdf) as doc:
        return [page.get_text() for page in doc]


_JXA_TEXT = r'''
ObjC.import('PDFKit');
function run(argv) {
  var d = $.PDFDocument.alloc.initWithURL($.NSURL.fileURLWithPath(argv[0]));
  if (!d || d.isNil()) { return 'ERR:cannot open PDF'; }
  var out = [];
  for (var i = 0; i < d.pageCount; i++) {
    var s = d.pageAtIndex(i).string;
    out.push((s && !s.isNil()) ? s.js : '');
  }
  return JSON.stringify(out);
}
'''

_JXA_RENDER = r'''
ObjC.import('PDFKit');
ObjC.import('AppKit');
function run(argv) {
  var d = $.PDFDocument.alloc.initWithURL($.NSURL.fileURLWithPath(argv[0]));
  if (!d || d.isNil()) { return 'ERR:cannot open PDF'; }
  var idx = parseInt(argv[1], 10), dpi = parseFloat(argv[3]);
  if (idx < 0 || idx >= d.pageCount) { return 'ERR:page out of range'; }
  var page = d.pageAtIndex(idx);
  var box = page.boundsForBox($.kPDFDisplayBoxMediaBox);
  var k = dpi / 72.0;
  var img = page.thumbnailOfSizeForBox(
      $.NSMakeSize(box.size.width * k, box.size.height * k),
      $.kPDFDisplayBoxMediaBox);
  var rep = $.NSBitmapImageRep.imageRepWithData(img.TIFFRepresentation);
  var png = rep.representationUsingTypeProperties($.NSBitmapImageFileTypePNG, $());
  return png.writeToFileAtomically(argv[2], true) ? 'OK' : 'ERR:write failed';
}
'''


def _osascript(script, args):
    if sys.platform != 'darwin' or not shutil.which('osascript'):
        raise NoBackend('PDFKit is macOS only')
    r = subprocess.run(['osascript', '-l', 'JavaScript', '-e', script, *args],
                       capture_output=True)
    out = r.stdout.decode('utf-8', 'replace').strip()
    if r.returncode != 0:
        raise NoBackend('osascript failed: '
                        + r.stderr.decode('utf-8', 'replace').strip())
    if out.startswith('ERR:'):
        raise ValueError(out[4:])
    return out


def _text_pdfkit(pdf):
    return json.loads(_osascript(_JXA_TEXT, [os.path.abspath(pdf)]))


# --- render: writes page `n` (1-based) of `pdf` to the PNG `out` ----------

def _render_poppler(pdf, n, out, dpi):
    exe = shutil.which('pdftoppm')
    if not exe:
        raise NoBackend('pdftoppm not on PATH')
    prefix = out[:-4] if out.lower().endswith('.png') else out
    r = subprocess.run([exe, '-r', str(dpi), '-f', str(n), '-l', str(n),
                        '-png', '-singlefile', pdf, prefix], capture_output=True)
    if r.returncode != 0:
        raise ValueError(r.stderr.decode('utf-8', 'replace').strip()
                         or f'pdftoppm exited {r.returncode}')
    if prefix + '.png' != out:
        os.replace(prefix + '.png', out)


def _render_pymupdf(pdf, n, out, dpi):
    try:
        import fitz
    except ImportError:
        raise NoBackend('PyMuPDF (fitz) not installed')
    with fitz.open(pdf) as doc:
        if not 1 <= n <= len(doc):
            raise ValueError('page out of range')
        doc[n - 1].get_pixmap(dpi=dpi).save(out)


def _render_pdfkit(pdf, n, out, dpi):
    _osascript(_JXA_RENDER, [os.path.abspath(pdf), str(n - 1),
                             os.path.abspath(out), str(dpi)])


TEXT = {'poppler': _text_poppler, 'pypdf': _text_pypdf,
        'pymupdf': _text_pymupdf, 'pdfkit': _text_pdfkit}
RENDER = {'poppler': _render_poppler, 'pymupdf': _render_pymupdf,
          'pdfkit': _render_pdfkit}


def _first(table, backend, *args):
    """Run the first available backend in `table`; return (name, result)."""
    order = BACKENDS if backend == 'auto' else (backend,)
    missing = []
    for name in order:
        fn = table.get(name)
        if fn is None:
            missing.append(f'{name}: cannot do this')
            continue
        try:
            return name, fn(*args)
        except NoBackend as e:
            missing.append(f'{name}: {e}')
    raise NoBackend('; '.join(missing))


def parse_pages(spec, count):
    """'3,5-7' -> [3, 5, 6, 7], each checked against the page count."""
    pages = []
    for part in spec.split(','):
        part = part.strip()
        m = re.fullmatch(r'(\d+)(?:-(\d+))?', part)
        if not m:
            raise ValueError(f'bad page spec {part!r} (want e.g. 3,5-7)')
        lo, hi = int(m.group(1)), int(m.group(2) or m.group(1))
        if lo < 1 or hi < lo or hi > count:
            raise ValueError(f'page(s) {part} outside 1-{count}')
        pages.extend(range(lo, hi + 1))
    return pages


def main(argv=None) -> int:
    parser = argparse.ArgumentParser(
        description="Find pages, print page text or render a page of a PDF, "
                    "with poppler, pypdf, PyMuPDF or macOS PDFKit -- whichever "
                    "is present.")
    parser.add_argument('pdf', help='the PDF file')
    what = parser.add_mutually_exclusive_group(required=True)
    what.add_argument('--find', nargs='+', metavar='REGEX',
                      help='print "page N: line" for lines matching any '
                           'pattern (case-insensitive)')
    what.add_argument('--pages', metavar='SPEC',
                      help='print the text of these pages, e.g. 3,5-7')
    what.add_argument('--render', type=int, metavar='N',
                      help='write page N as a PNG to --out')
    parser.add_argument('--out', help='PNG path for --render')
    parser.add_argument('--dpi', type=int, default=200,
                        help='--render resolution (default 200)')
    parser.add_argument('--backend', choices=('auto',) + BACKENDS,
                        default='auto', help='force one backend (default: '
                        'the first available, in the order listed)')
    args = parser.parse_args(argv)

    if not os.path.isfile(args.pdf):
        print(f'pdf_text: no such file: {args.pdf}', file=sys.stderr)
        return 2
    install_hint = ('install poppler (brew/apt install poppler-utils) or '
                    '`pip install pypdf`')
    try:
        if args.render is not None:
            if not args.out:
                parser.error('--render needs --out FILE.png')
            if args.render < 1:
                parser.error('--render pages are numbered from 1')
            name, _ = _first(RENDER, args.backend, args.pdf, args.render,
                             args.out, args.dpi)
            print(f'pdf_text: backend {name}', file=sys.stderr)
            if not os.path.isfile(args.out) or not os.path.getsize(args.out):
                print(f'pdf_text: {name} wrote no image', file=sys.stderr)
                return 2
            print(args.out)
            return 0

        name, pages = _first(TEXT, args.backend, args.pdf)
        print(f'pdf_text: backend {name}, {len(pages)} page(s)', file=sys.stderr)
        if args.pages:
            for n in parse_pages(args.pages, len(pages)):
                print(f'=== page {n} ===')
                print(pages[n - 1].rstrip())
            return 0
        pats = [re.compile(p, re.I) for p in args.find]
        hits = 0
        for n, text in enumerate(pages, 1):
            for line in text.splitlines():
                if any(p.search(line) for p in pats):
                    print(f'page {n}: {line.strip()}')
                    hits += 1
        if not hits:
            print('no match (a scanned PDF has no text layer: render pages '
                  'instead)')
        return 0
    except NoBackend as e:
        hint = ('install poppler or PyMuPDF' if args.render is not None
                else install_hint)
        print(f'pdf_text: no backend could do this ({e}); {hint}',
              file=sys.stderr)
        return 3
    except (ValueError, re.error) as e:
        print(f'pdf_text: {e}', file=sys.stderr)
        return 2
    except Exception as e:                                      # noqa: BLE001
        # A backend that is present but cannot parse THIS file.
        print(f'pdf_text: could not read {args.pdf}: {type(e).__name__}: {e}',
              file=sys.stderr)
        return 2


if __name__ == '__main__':
    sys.exit(main())
