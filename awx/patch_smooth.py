#!/usr/bin/env python3
"""One-shot patch: integrate repo smooth_octolinear_chains (#536) into
topo_emit.py."""
import os

HERE = os.path.dirname(os.path.abspath(__file__))
p = os.path.join(HERE, 'topo_emit.py')
t = open(p).read()

t = t.replace("import argparse\nimport math",
              "import argparse\nimport math\nimport re")

t = t.replace("""    ap.add_argument('--dump-segs', default='')""",
              """    ap.add_argument('--dump-segs', default='')
    ap.add_argument('--no-smooth', action='store_true',
                    help='skip the repo #536 octolinear smoothing pass')""")

t = t.replace('''def main():
    ap = argparse.ArgumentParser()''',
'''def strip_net_segments(txt, net_ids):
    """Remove every (segment ...) block whose (net N) is in net_ids,
    paren-balanced (handles both KiCad multi-line and our one-line
    forms)."""
    out = []
    i = 0
    while True:
        j = txt.find('(segment', i)
        if j < 0:
            out.append(txt[i:])
            break
        k, depth = j, 0
        while True:
            c = txt[k]
            if c == '(':
                depth += 1
            elif c == ')':
                depth -= 1
                if depth == 0:
                    break
            k += 1
        m = re.search(r'\\(net (\\d+)\\)', txt[j:k + 1])
        if m and int(m.group(1)) in net_ids:
            out.append(txt[i:j].rstrip(' \\t'))
            e = k + 1
            if e < len(txt) and txt[e] == '\\n':
                e += 1
            i = e
        else:
            out.append(txt[i:k + 1])
            i = k + 1
    return ''.join(out)


def main():
    ap = argparse.ArgumentParser()''')

old_writer = '''    print('\\nverification:')
    bad, _off = run_verify()
    print(f'  {"CLEAN" if not bad else str(bad) + " violations"}')

    # write board
    txt = open(a.board, encoding='utf-8').read()
    add = []
    for nm in names:
        nid, _ = byname[nm]
        for (p, q, layer) in out_segs[nm]:
            add.append(
                f'  (segment (start {p[0]:.4f} {p[1]:.4f}) '
                f'(end {q[0]:.4f} {q[1]:.4f}) (width {TRACK}) '
                f'(layer "{layer}") (net {nid}))\\n')
        for (vx, vy) in out_vias[nm]:
            add.append(
                f'  (via (at {vx:.4f} {vy:.4f}) (size {VIA_SIZE}) '
                f'(drill {VIA_DRILL}) (layers "F.Cu" "B.Cu") '
                f'(net {nid}))\\n')
        for (p, q, _l) in out_segs[nm]:
            add.append(
                f'  (gr_line (start {p[0]:.4f} {p[1]:.4f}) '
                f'(end {q[0]:.4f} {q[1]:.4f}) '
                f'(stroke (width 0.05) (type solid)) '
                f'(layer "Eco2.User"))\\n')
    k = txt.rstrip().rfind(')')'''

new_writer = '''    print('\\nverification:')
    bad, _off = run_verify()
    print(f'  {"CLEAN" if not bad else str(bad) + " violations"}')

    # repo octolinear smoothing (#536): collapse the distributed 45
    # nudges into single elbows and merge our launch legs with the
    # fanout stubs. Every collapse is clearance-validated against ALL
    # copper and connectivity-guarded per net; it splices
    # pcb_data.segments in place, so the final geometry is read back
    # from there.
    smoothed = False
    if not a.no_smooth:
        from kicad_parser import Segment, Via
        from pcb_modification import smooth_octolinear_chains
        pre_len = {nm: sum(math.hypot(q[0] - p[0], q[1] - p[1])
                           for (p, q, _l) in out_segs[nm]) for nm in names}
        for nm in names:
            nid, _ = byname[nm]
            for (p, q, layer) in out_segs[nm]:
                pcb.segments.append(Segment(p[0], p[1], q[0], q[1],
                                            TRACK, layer, nid))
            for (vx, vy) in out_vias[nm]:
                pcb.vias.append(Via(vx, vy, VIA_SIZE, VIA_DRILL,
                                    ['F.Cu', 'B.Cu'], nid))
        _n, _nets, _rm, _addl, st = smooth_octolinear_chains(
            [], pcb, kids, clearance=0.1)
        final_segs = {}
        for nm in names:
            nid, _ = byname[nm]
            final_segs[nm] = [s for s in pcb.segments if s.net_id == nid]
        post_len = {nm: sum(math.hypot(s.end_x - s.start_x,
                                       s.end_y - s.start_y)
                            for s in final_segs[nm]) for nm in names}
        print(f'\\nsmooth_octolinear_chains (#536): '
              f'{st.get("spans", 0)} spans on {_nets} nets, '
              f'-{st.get("saved_mm", 0):.2f} mm; segments '
              f'{sum(len(s) for s in out_segs.values())} -> '
              f'{sum(len(s) for s in final_segs.values())}; length '
              f'{sum(pre_len.values()):.2f} -> '
              f'{sum(post_len.values()):.2f} mm')
        smoothed = True

    # write board
    txt = open(a.board, encoding='utf-8').read()
    add = []
    if smoothed:
        txt = strip_net_segments(txt, kids)
        for nm in names:
            nid, _ = byname[nm]
            for s in final_segs[nm]:
                add.append(
                    f'  (segment (start {s.start_x:.4f} {s.start_y:.4f}) '
                    f'(end {s.end_x:.4f} {s.end_y:.4f}) '
                    f'(width {s.width}) '
                    f'(layer "{s.layer}") (net {nid}))\\n')
            for (vx, vy) in out_vias[nm]:
                add.append(
                    f'  (via (at {vx:.4f} {vy:.4f}) (size {VIA_SIZE}) '
                    f'(drill {VIA_DRILL}) (layers "F.Cu" "B.Cu") '
                    f'(net {nid}))\\n')
            for s in final_segs[nm]:
                add.append(
                    f'  (gr_line (start {s.start_x:.4f} {s.start_y:.4f}) '
                    f'(end {s.end_x:.4f} {s.end_y:.4f}) '
                    f'(stroke (width 0.05) (type solid)) '
                    f'(layer "Eco2.User"))\\n')
    else:
        for nm in names:
            nid, _ = byname[nm]
            for (p, q, layer) in out_segs[nm]:
                add.append(
                    f'  (segment (start {p[0]:.4f} {p[1]:.4f}) '
                    f'(end {q[0]:.4f} {q[1]:.4f}) (width {TRACK}) '
                    f'(layer "{layer}") (net {nid}))\\n')
            for (vx, vy) in out_vias[nm]:
                add.append(
                    f'  (via (at {vx:.4f} {vy:.4f}) (size {VIA_SIZE}) '
                    f'(drill {VIA_DRILL}) (layers "F.Cu" "B.Cu") '
                    f'(net {nid}))\\n')
            for (p, q, _l) in out_segs[nm]:
                add.append(
                    f'  (gr_line (start {p[0]:.4f} {p[1]:.4f}) '
                    f'(end {q[0]:.4f} {q[1]:.4f}) '
                    f'(stroke (width 0.05) (type solid)) '
                    f'(layer "Eco2.User"))\\n')
    k = txt.rstrip().rfind(')')'''

assert old_writer in t, 'writer anchor not found'
t = t.replace(old_writer, new_writer)
open(p, 'w').write(t)
print('smoothing integrated')
