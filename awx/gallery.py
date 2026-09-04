#!/usr/bin/env python3
"""gallery.py OUT.html TITLE [--section 'heading' IMG [IMG...]]... : one
HTML page of renders (data-URI PNGs, one <figure> each, captions from
the file name), for a look-by-eye at diagnose_open's pictures."""
import base64
import html
import os
import sys

out, title = sys.argv[1], sys.argv[2]
sections, cur = [], None
for tok in sys.argv[3:]:
    if tok == '--section':
        cur = None
        continue
    if cur is None:
        cur = [tok, []]
        sections.append(cur)
        continue
    cur[1].append(tok)
parts = [f'<title>{html.escape(title)}</title>',
         '<style>body{background:#111;color:#ddd;font:14px system-ui;margin:0;padding:16px}'
         'h1{font-size:20px;margin:0 0 12px}h2{font-size:16px;margin:24px 0 8px;'
         'border-bottom:1px solid #333;padding-bottom:4px}figure{margin:0 0 18px}'
         'figcaption{color:#aaa;margin:4px 0 0;font-family:ui-monospace,monospace;font-size:12px}'
         'img{max-width:100%;display:block;border:1px solid #333;border-radius:4px}'
         '.note{color:#bbb;max-width:80ch;line-height:1.45}</style>',
         f'<h1>{html.escape(title)}</h1>']
for head, imgs in sections:
    parts.append(f'<h2>{html.escape(head)}</h2>')
    for p in imgs:
        if not os.path.exists(p):
            parts.append(f'<p class="note">missing: {html.escape(p)}</p>')
            continue
        b = base64.b64encode(open(p, 'rb').read()).decode()
        parts.append(f'<figure><img src="data:image/png;base64,{b}">'
                     f'<figcaption>{html.escape(os.path.basename(p))}</figcaption></figure>')
open(out, 'w').write('\n'.join(parts))
print(out, os.path.getsize(out) // 1024, 'KB')
