#!/usr/bin/env python3
"""Power view of a board: the power components first -- each part that passes
power from rail to rail (regulator, filter, fuse, switch), each connector
where power enters, each decap and the rest of the power groups, filled in a
rail's colour, outlined and named -- over each supply rail's copper in the
same colour, muted, with every other net's copper faint.

    python3 -X utf8 py_tools/render_power.py routed.kicad_pcb [-o routed_power.png]
        [--groups groups.json] [--manifest redo_commands.sh] [--power-nets NET ...]

Which nets are rails, in order: `--power-nets`, else the `--power-nets` of a
recorded chain (`--manifest`, a redo_commands.sh), else the nets
`list_nets.find_power_nets` names; ground is never a rail. A rail is labelled
a plane when the manifest's route_planes step pours it (else when it has a
zone). `--groups` adds the parts of the groups file's power groups (role or
name `power`) to the named parts.

Which parts are the power components (a picture, not a grade):
  * a part passes power when one of its rail pins is an output (`OUT`, `VO`,
    `SW`, `LX`), or it is a two-pad part between rails, a switch, jumper,
    fuse, diode, resistor, inductor, bead or transistor, or a small (8 pads or
    fewer) or power-group chip whose pins carry no names; a chip with supply
    pins alone on several rails (a level shifter) is a load. Its input is the
    rail on an input pin, else the rail whose name gives a voltage or says
    VBUS/VIN/BAT, the higher voltage first; it is filled in its OUTPUT rail's
    colour;
  * power enters at a connector on a rail that some part takes in and no part
    makes; it is filled in that rail's colour;
  * a decap is filled in the rail it decouples; another part of a power group
    in the first rail it touches.
"""
#: #937 registry: which door(s) show this tool, and whether it changes
#: the board. Read by krt_registry.py -- by AST, never imported.
KRT_TOOL = {'scope': ['placement', 'routing'], 'kind': 'instrument'}

import _path  # noqa: F401  (py_tools -> py_router/py_placer on sys.path)

import argparse
import math
import os
import re
import shlex
import sys

OUT_PIN = re.compile(r'OUT|\bSW\d*\b|^LX|\bVO\b')
IN_PIN = re.compile(r'^(VIN|IN|VI|PVIN|AVIN|VBUS|VCC_?IN)')
PASS = ('L', 'FB', 'F', 'D', 'R', 'SW', 'JP', 'K', 'Q', 'S')
CONN = ('J', 'P', 'CN', 'USB', 'BT', 'X', 'TP')
CHIP = ('U', 'IC', 'M')


def prefix(ref):
    return ref.rstrip('0123456789~').upper()


def chain_nets(manifest):
    """(power nets, plane nets) a recorded chain names: every step's
    `--power-nets`, and the nets its route_planes step pours."""
    power, planes = [], set()
    for line in open(manifest, encoding='utf-8', errors='replace'):
        if line.lstrip().startswith('#') or '--help' in line or 'python' not in line:
            continue
        try:
            toks = shlex.split(line)
        except ValueError:
            continue
        for flag in ('--power-nets', '--nets'):
            if flag not in toks or (flag == '--nets' and 'route_planes.py' not in line):
                continue
            for x in toks[toks.index(flag) + 1:]:
                if x.startswith('--'):
                    break
                if flag == '--nets':
                    # #1495: a split layer's '|' group pours each of its nets
                    from plane_groups import split_plane_group
                    planes.update(split_plane_group(x))
                elif x not in power:
                    power.append(x)
    return power, planes


def volts(name):
    """The voltage a rail's name says (3V3, +5V, 1.8V; VBUS 5, VBAT 3.7), or
    None."""
    nm = name.upper().split('/')[-1]
    m = re.search(r'(\d+)V(\d+)', nm)
    if m:
        return float(f"{m.group(1)}.{m.group(2)}")
    m = re.search(r'(\d+(?:\.\d+)?)V', nm)
    if m:
        return float(m.group(1))
    if any(k in nm for k in ('VBUS', 'USB', 'VCHRG', 'VIN', 'VEXT')):
        return 5.0
    if 'BAT' in nm:
        return 3.7
    return None


def inputish(name):
    return bool(re.search(r'VBUS|VIN|BAT|CHRG|VEXT|USB|\bDC\b', name.upper()))


def power_model(pcb, rails, power_parts=()):
    """What the picture draws, from the board and its rails (net ids):
    {'on': {ref: rails it touches}, 'flow': {part: (input rails, output
    rails)}, 'entries': connectors where power enters, 'decaps': {cap: rail}}."""
    from placement.grouping import BoardView
    rail_set = set(rails)
    on = {r: {p.net_id for p in fp.pads if p.net_id in rail_set} for r, fp in pcb.footprints.items()}

    def pads_on(ref, n):
        return [p for p in pcb.footprints[ref].pads if p.net_id == n]

    def orient(b):
        outs, ins = set(), set()
        for n in on[b]:
            fns = [(q.pinfunction or '').upper() for q in pads_on(b, n)]
            if any(OUT_PIN.search(f) for f in fns):
                outs.add(n)
            elif any(IN_PIN.search(f) for f in fns):
                ins.add(n)
        if outs:
            return on[b] - outs, outs
        live = [q for q in pcb.footprints[b].pads if q.net_id]
        unnamed = not any((q.pinfunction or '').strip() for q in pcb.footprints[b].pads)
        if not (len(live) == 2 or prefix(b) in PASS
                or (unnamed and (b in power_parts or len(live) <= 8))):
            return None
        name = lambda n: pcb.nets[n].name  # noqa: E731
        rank = sorted(on[b], key=lambda n: (not inputish(name(n)), -(volts(name(n)) or 0), name(n)))
        known = [n for n in rank if volts(name(n)) is not None or inputish(name(n))]
        if known and len(known) < len(rank):
            return set(known), set(rank) - set(known)
        return {rank[0]}, set(rank[1:])

    flow = {}
    for b in sorted(r for r, s in on.items() if len(s) >= 2 and prefix(r) not in CONN):
        got = orient(b)
        if got and got[0] and got[1]:
            flow[b] = got
    produced = {n for _i, o in flow.values() for n in o}
    taken = {n for i, _o in flow.values() for n in i}
    conns = sorted(r for r, s in on.items() if s and prefix(r) in CONN)
    entries = {c for c in conns if any(n in taken and n not in produced for n in on[c])}

    decaps = {c: n for c, n in BoardView(pcb).decaps().items() if n in rail_set and c not in flow}
    return {'on': on, 'flow': flow, 'entries': entries, 'decaps': decaps}


def render(pcb, rails, planes, model, power_parts=(), title='', size=1800, theme='dark'):
    """The picture, the key below the board."""
    from PIL import Image, ImageDraw
    from render_chrome import draw_mark
    from render_placement import group_palette
    from route_render import BoardRenderer, load_font
    rail_set = set(rails)
    colour = dict(zip(rails, group_palette(len(rails))))
    r = BoardRenderer(pcb, size=size, supersample=1, theme=theme, zone_tint=0.10, layer_alpha=60)
    th, tf = r.theme, r.tf
    img = r.frame(segments=[s for s in pcb.segments if s.net_id not in rail_set
                            and not getattr(s, 'graphic', False)],
                  vias=[v for v in pcb.vias if v.net_id not in rail_set]).convert('RGBA')
    pour = Image.new('RGBA', img.size, (0, 0, 0, 0))
    dp = ImageDraw.Draw(pour)
    for z in pcb.zones:
        if z.net_id in rail_set and len(z.polygon) >= 3:
            c = colour[z.net_id]
            dp.polygon([tf.pt(x, y) for x, y in z.polygon], fill=c + (60,), outline=c + (200,))
    img = Image.alpha_composite(img, pour).convert('RGB')
    d = ImageDraw.Draw(img)
    body = th.rgb('board_body')
    mute = {n: tuple(int(body[k] * 0.45 + colour[n][k] * 0.55) for k in range(3)) for n in rails}
    for s in pcb.segments:
        if s.net_id in rail_set:
            d.line([tf.pt(s.start_x, s.start_y), tf.pt(s.end_x, s.end_y)], fill=mute[s.net_id],
                   width=max(1, int(round(tf.length(s.width) * 0.7))), joint='curve')
    for v in pcb.vias:
        if v.net_id in rail_set:
            cx, cy = tf.pt(v.x, v.y)
            rr = max(1.5, tf.length(v.size) / 2)
            d.ellipse([cx - rr, cy - rr, cx + rr, cy + rr], fill=mute[v.net_id])

    def box(ref, m):
        ps = pcb.footprints[ref].pads
        if not ps:
            return None
        (x0, y0) = tf.pt(min(p.global_x - p.size_x / 2 for p in ps) - m,
                         min(p.global_y - p.size_y / 2 for p in ps) - m)
        (x1, y1) = tf.pt(max(p.global_x + p.size_x / 2 for p in ps) + m,
                         max(p.global_y + p.size_y / 2 for p in ps) + m)
        return [x0, y0, x1, y1]
    # the components: a rail's colour for each, the pads showing through
    flow, entries, decaps, on = model['flow'], model['entries'], model['decaps'], model['on']
    tint = {}
    for b, (_ins, outs) in flow.items():
        tint[b] = colour[min(outs, key=lambda n: rails.index(n))]
    for e in entries:
        tint[e] = colour[min(on[e], key=lambda n: rails.index(n))]
    for c, n in decaps.items():
        tint[c] = colour[n]
    for ref in power_parts:
        if ref not in tint and on.get(ref):
            tint[ref] = colour[min(on[ref], key=lambda n: rails.index(n))]
    fill = Image.new('RGBA', img.size, (0, 0, 0, 0))
    df = ImageDraw.Draw(fill)
    boxes = {}
    for ref, c in tint.items():
        b = box(ref, 0.25 if ref in decaps else 0.4)
        if b:
            boxes[ref] = b
            df.rectangle(b, fill=c + (150,))
    img = Image.alpha_composite(img.convert('RGBA'), fill).convert('RGB')
    d = ImageDraw.Draw(img)
    hot = th.rgb('status_best')
    named = set(flow) | set(entries)
    for ref in sorted(set(power_parts) - set(tint)):
        b = box(ref, 0.4)
        if b:
            d.rectangle(b, outline=th.rgb('chrome_text'), width=2)
    for ref, b in boxes.items():
        d.rectangle(b, outline=hot if ref in named else (255, 255, 255),
                    width=4 if ref in named else 2)
    font = load_font(max(13, r.H // 62))
    small = load_font(max(10, r.H // 95))
    for ref in sorted(boxes, key=lambda q: q in named):
        b = boxes[ref]
        val = (getattr(pcb.footprints[ref], 'value', '') or '').strip()
        txt, f = (f"{ref} {val}"[:24], font) if ref in named else (ref, small)
        x, y = b[2] + 3, b[1]
        d.rectangle([x - 2, y - 2, x + d.textlength(txt, font=f) + 2, y + f.size + 3], fill=(0, 0, 0))
        d.text((x, y), txt, fill=hot if ref in named else (235, 235, 235), font=f)
    if title:
        r._label(img, title)
    rows = [(colour[n], 'solid', pcb.nets[n].name + (' (plane)' if pcb.nets[n].name in planes else ''))
            for n in rails]
    rows += [(hot, 'ring', 'regulator, filter, switch, entry'), ((255, 255, 255), 'ring', 'decap, other power part')]
    kf = load_font(max(12, r.H // 75))
    sw = max(10, r.H // 90)
    width = max(sw + 8 + d.textlength(t, font=kf) for _, _, t in rows)
    cols = max(1, min(len(rows), int((r.W - 16) // (width + 16))))
    lh = sw + 8
    out = Image.new('RGB', (img.width, img.height + math.ceil(len(rows) / cols) * lh + 12),
                    th.rgb('chrome_band'))
    out.paste(img, (0, 0))
    dk = ImageDraw.Draw(out)
    slot = (r.W - 16) / cols
    for i, (rgb, mark, txt) in enumerate(rows):
        x, y = int(8 + (i % cols) * slot), int(img.height + 6 + (i // cols) * lh)
        draw_mark(dk, [x, y, x + sw, y + sw], rgb, mark)
        dk.text((x + sw + 6, y - 2), txt, fill=th.rgb('chrome_text'), font=kf)
    return out


def main(argv=None):
    ap = argparse.ArgumentParser(description=__doc__.split('\n\n')[0],
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('board', help='the board (a routed one shows the rails\' copper)')
    ap.add_argument('-o', '--out', help='the PNG (default: beside the board, <name>_power.png)')
    ap.add_argument('--groups', help='a groups file: its power groups\' parts are outlined too')
    ap.add_argument('--manifest', help="a recorded chain's redo_commands.sh: its --power-nets are "
                                       "the rails, its route_planes --nets the planes")
    ap.add_argument('--power-nets', nargs='+', metavar='NET', help='the rails, by name (ground '
                                                                   'is left out)')
    ap.add_argument('--title', help='the caption (default: the board\'s name)')
    ap.add_argument('--size', type=int, default=1800, help='longest side of the board, px (default 1800)')
    ap.add_argument('--theme', default='dark', help='dark (default) or light')
    a = ap.parse_args(argv)
    from kicad_parser import parse_kicad_pcb
    from list_nets import find_power_nets
    from net_queries import is_ground_net_name
    pcb = parse_kicad_pcb(a.board)
    power, planes = chain_nets(a.manifest) if a.manifest else ([], set())
    if a.power_nets:
        power = a.power_nets
    elif not power:
        power = [name for name, _n in find_power_nets(pcb)[1]]
    if not a.manifest:
        planes = {pcb.nets[z.net_id].name for z in pcb.zones if z.net_id in pcb.nets}
    ids = {n.name: i for i, n in pcb.nets.items() if n.name}
    rails = sorted({ids[n] for n in power if n in ids and not is_ground_net_name(n)},
                   key=lambda n: (-len(pcb.nets[n].pads), pcb.nets[n].name))
    if not rails:
        print(f"render_power: no power rail found on {a.board}; name them with --power-nets",
              file=sys.stderr)
        return 2
    power_parts = set()
    if a.groups:
        from placement.grouping import load_groups
        power_parts = {r for g in load_groups(a.groups)['groups']
                       if g.get('role') == 'power' or 'power' in g['name']
                       for r in g['refs'] if r in pcb.footprints}
    model = power_model(pcb, rails, power_parts)
    out = a.out or os.path.splitext(a.board)[0] + '_power.png'
    render(pcb, rails, planes, model, power_parts,
           title=a.title or os.path.splitext(os.path.basename(a.board))[0],
           size=a.size, theme=a.theme).save(out)
    print(f"render_power: {out} ({len(rails)} rails, {len(model['flow'])} parts passing power, "
          f"{len(model['entries'])} entries, {len(model['decaps'])} decaps, "
          f"{len(power_parts)} power-group parts)")
    return 0


if __name__ == '__main__':
    sys.exit(main())
