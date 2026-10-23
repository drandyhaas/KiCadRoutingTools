#!/usr/bin/env python3
"""Remove ONLY routing output from a .kicad_pcb, preserving everything else.

Makes the unrouted twin of a routed corpus board, which is what a
perturbed-corpus run needs as a subject: the placement CLIs refuse a board
carrying copper (`place_pose` and `check_floorplan` without
`--allow-routed`; the plan-pcb-placement skill's
`make_unplaced.py` outright), and on a ROUTED board that copper also
encodes the original poses (run 14 measured 301 of 569 pads within 5 um of
their own track endpoint, which `fence_audit` cannot see because it
compares poses and never opens copper).

    python3 -X utf8 tests/stress/strip_copper_only.py SRC.kicad_pcb DST.kicad_pcb

Why this exists instead of `tests/stress/strip_routing.py` / `prep_set2.py`:
those are corpus NORMALIZERS and they rewrite `Edge.Cuts` (RDP-simplify the
outline, rebuild it as chained segments, drop graphics). The routing skill
forbids them for exactly that reason -- the outline is a mechanical decision.
For a recovery run the outline must stay bit-identical, because the final board
is compared against a human reference that has the ORIGINAL outline.

So this removes, and removes nothing else:
  * top-level `(segment ...)`  -- straight track copper
  * top-level `(arc ...)`      -- curved track copper (this board is arc-heavy)
  * top-level `(via ...)`
  * `(filled_polygon ...)` inside zones -- the pour result, not the pour intent
  * top-level zones whose `(attr (teardrop ...))` marks them as TEARDROPS
    (#1164). KiCad generates them where a track meets a pad or via and
    regenerates them on every refill (a stripped board refilled by kicad-cli
    keeps none), so they are routing output, and each one sits on a human pad
    or via: KiCad's complex_hierarchy demo carries 165 that locate all 68 of
    its pad-bearing parts. The parser reads them as ordinary signal-net zones.
  * top-level `(generated ...)` forms none of whose `members` survives -- a
    length-tuning pattern is routing output whatever else it says: its
    `base_line`, `origin` and `end` are the human track's coordinates, and its
    members are the tracks, arcs and vias deleted above (#1164).
  * top-level `(group ...)` forms that HAD members and lost every one of them
    to the drops above (a group holding only tracks, or only a tuning
    pattern). A group that keeps any member is kept as it is: KiCad's loader
    ignores a member uuid that names nothing.

Zone DEFINITIONS survive (pours, keep-outs, rule areas): a zone is design
intent (which net pours where), not routing output, and dropping it would
silently delete part of the board's design. Footprints, pads, graphics,
`Edge.Cuts`, `setup`, stackup, netlist and every surviving uuid are untouched.

The scanner is quote-aware: KiCad s-expr strings may contain parentheses, and a
naive depth counter desyncs on the first `(gr_text "foo (bar)")`.
"""
import os
import re
import shutil
import sys

DROP_TOP = {'segment', 'arc', 'via'}
DROP_NESTED = {'filled_polygon'}
#: Top-level forms that NAME other items in a `(members ...)` child.
MEMBERED = {'generated', 'group'}
#: The child carrying an item's own id: `uuid` (KiCad 8+), `tstamp` (KiCad
#: 6/7 items), `id` (KiCad 6 groups).
ID_HEADS = ('uuid', 'tstamp', 'id')
_HEAD = re.compile(r'\(\s*([A-Za-z_0-9]+)')
_ATOM = re.compile(r'"((?:[^"\\]|\\.)*)"|([^\s()"]+)')


def _forms(text):
    """Yield (start, end, depth, head, parent_start) for every s-expression form.

    Forms come innermost-first (in closing order); `parent_start` is the start
    offset of the enclosing form, None at the top of the file.
    """
    stack = []
    in_str = False
    esc = False
    for i, ch in enumerate(text):
        if in_str:
            if esc:
                esc = False
            elif ch == '\\':
                esc = True
            elif ch == '"':
                in_str = False
            continue
        if ch == '"':
            in_str = True
        elif ch == '(':
            stack.append(i)
        elif ch == ')':
            if not stack:
                raise SystemExit('unbalanced ")" at offset %d' % i)
            s = stack.pop()
            m = _HEAD.match(text, s)
            yield (s, i + 1, len(stack), (m.group(1) if m else ''),
                   stack[-1] if stack else None)
    if stack:
        raise SystemExit('unbalanced "(" -- %d unclosed' % len(stack))


def _atoms(text, form):
    """The bare tokens of a LEAF form after its head: `(members "a" b)` -> [a, b]."""
    s, e, _depth, head, _parent = form
    body = text[s + 1:e - 1]
    body = body[body.index(head) + len(head):]
    return [m.group(1) if m.group(1) is not None else m.group(2)
            for m in _ATOM.finditer(body)]


def strip(text):
    """Return (stripped text, counts by kind of dropped form)."""
    # Keep only the forms the rules read: a monster board has millions.
    top, cuts = [], []   # cuts: (start, end, kind)
    children = {}        # parent start -> the child forms the rules read
    for f in _forms(text):
        s, e, depth, head, parent = f
        if depth == 1 and (head in DROP_TOP or head in MEMBERED or head == 'zone'):
            top.append(f)
        elif depth >= 2 and head in DROP_NESTED:
            cuts.append((s, e, 'fill'))
        if (depth == 2 and (head in ID_HEADS or head in ('attr', 'members'))) \
                or (depth == 3 and head == 'teardrop'):
            children.setdefault(parent, []).append(f)

    def kids(form, head):
        return [c for c in children.get(form[0], ()) if c[3] == head]

    def own_id(form):
        for c in children.get(form[0], ()):
            if c[3] in ID_HEADS:
                atoms = _atoms(text, c)
                if atoms:
                    return atoms[0]
        return None

    def is_teardrop(zone):
        return any(kids(a, 'teardrop') for a in kids(zone, 'attr'))

    deleted = set()  # ids of every dropped item, for the membership rules
    for f in top:
        s, e, _depth, head, _parent = f
        if head in DROP_TOP:
            kind = 'copper'
        elif head == 'zone' and is_teardrop(f):
            kind = 'teardrop'
        else:
            continue
        cuts.append((s, e, kind))
        uid = own_id(f)
        if uid:
            deleted.add(uid)

    # A generator or group can name another one, so iterate to a fixpoint: a
    # group whose only member is a tuning pattern goes once the pattern does.
    pending = [f for f in top if f[3] in MEMBERED]
    changed = True
    while changed:
        changed = False
        for f in list(pending):
            members = [m for mf in kids(f, 'members') for m in _atoms(text, mf)]
            survivors = [m for m in members if m not in deleted]
            if survivors or (f[3] == 'group' and not members):
                continue
            pending.remove(f)
            cuts.append((f[0], f[1], f[3]))
            uid = own_id(f)
            if uid:
                deleted.add(uid)
            changed = True

    # Forms are yielded innermost-first; keep only outermost non-overlapping.
    cuts.sort()
    merged = []
    for s, e, kind in cuts:
        if merged and s < merged[-1][1]:
            merged[-1] = (merged[-1][0], max(merged[-1][1], e), merged[-1][2])
        else:
            merged.append((s, e, kind))
    out = []
    prev = 0
    for s, e, _kind in merged:
        # also swallow the run of whitespace that indented the dropped form
        b = s
        while b > prev and text[b - 1] in ' \t':
            b -= 1
        if b > prev and text[b - 1] == '\n':
            b -= 1
        out.append(text[prev:b])
        prev = e
    out.append(text[prev:])
    counts = {k: 0 for k in ('copper', 'fill', 'teardrop', 'generated', 'group')}
    for _s, _e, kind in merged:
        counts[kind] += 1
    return ''.join(out), counts


def main(src, dst):
    text = open(src, encoding='utf-8').read()
    new, n = strip(text)
    os.makedirs(os.path.dirname(os.path.abspath(dst)), exist_ok=True)
    with open(dst, 'w', encoding='utf-8', newline='') as f:
        f.write(new)
    for ext in ('.kicad_pro', '.kicad_prl', '.kicad_dru'):
        sib = os.path.splitext(src)[0] + ext
        if os.path.exists(sib):
            shutil.copyfile(sib, os.path.splitext(dst)[0] + ext)
    print('dropped %d copper form(s) (segment/arc/via), %d zone fill(s), '
          '%d teardrop zone(s), %d generated form(s) (tuning patterns), '
          '%d group(s); %d -> %d chars'
          % (n['copper'], n['fill'], n['teardrop'], n['generated'], n['group'],
             len(text), len(new)))


if __name__ == '__main__':
    if len(sys.argv) != 3:
        raise SystemExit(__doc__.strip().splitlines()[2].strip())
    main(sys.argv[1], sys.argv[2])
