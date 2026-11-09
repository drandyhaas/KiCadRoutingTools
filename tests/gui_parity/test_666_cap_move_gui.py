#!/usr/bin/env python3
"""#666 on both fronts: the scoped cap move is applied, re-welded and kept the same way by the CLI and the Route tab.

    python3 tests/gui_parity/test_666_cap_move_gui.py

(re-execs into KiCad's bundled python automatically, like its siblings)

THE MECHANISM. net_rescue's permissive bare-ball retry ships an escape via that
lands on a MOVABLE cap, and queues the cap's verified relocation on
`pcb_data._pending_cap_moves`. The run that owns the artifact applies every
queued move at a SEQUENCE POINT -- after the write, after the plane finalize,
after each reconciliation lap: the cap moved, the model mirrored, its nets
re-welded by `oracle_reconnect`, the weld's nets handed to the #984 soft-joint
bridge. The CLI does it on its written file. The Route tab has no file: the
engine re-welds on a STAGED board (stage_board_fn: the live board plus the
change-set plus every queued move) and merges the weld's copper into the
change-set; the applier moves the live footprint, and re-welds itself only
when there was no staged board.

Spies and seams, the same on both fronts: `kicad_oracle.oracle_reconnect` is a
spy that records what each call was handed -- nets, config, kwargs, where C1
sits in the board FILE and in the model -- reports one weld on the moved cap's
nets (so the #984 bridge has something to bridge) and lays no copper;
`route._late_soft_joint_bridge984` is wrapped to record the nets it is given.

Arms 1-5 run on cap_chain (two 0402s, C1/C2, between J1 and J2) with a
pcbnew-authored B.Cu pour on DPA_N, so the plane finalize's oracle leg runs, a
power width, a class clearance and a .kicad_dru layer rule (every by-name map
the re-weld is handed has a value), routing DPA_P and DPA_N. Their move is
queued by WRAPPING the rescue (the real one still runs) exactly as net_rescue
spells it: C1, 0.7 mm east.

  1. CLI, a move the main pass queued (the reference): the written board has
     C1 at the target; one re-weld on C1's nets, before the finalize's oracle
     leg; every oracle call saw C1 moved; C1's nets reach the #984 bridge.
  2. GUI engine, with the tab's stage_board_fn: the re-weld runs IN-RUN, once,
     on a staged board with C1 at the target, with the CLI's config and kwargs,
     in the CLI's order (before the finalize's oracle leg); the model is
     mirrored before the finalize; C1's nets reach the #984 bridge;
     results_data['pending_cap_moves'] carries the move marked 'in-run'.
  3. GUI apply of arm 2 (the REAL RoutingDialog._apply_results_to_board_body
     on the live pcbnew board): C1 and its pads moved, C2 untouched, and NO
     second re-weld; the dialog's pcb_data agrees.
  4. GUI fallback, no stage_board_fn: no in-run re-weld and nothing for the
     bridge; the move is posted marked 'applier', and the apply moves C1 and
     re-welds once with the CLI's config and kwargs.
  5. A move queued INSIDE the plane finalize -- by the rescue of a real nested
     batch_route sub-run the finalize's repair runs (the seam wraps
     repair_planes.repair_planes to run one, sharing the board the finalize
     handed it; KICAD_FINALIZE_ONLY=1 with skip_routing leaves DPA_N for the
     finalize): the CLI moves C1 in its written board and re-welds after the
     finalize, with live reuse and without it (KICAD_PLANE_FINALIZE_LIVE=0,
     where the repair's board is its own); the GUI re-welds in-run after the
     finalize and its apply moves C1.
  6. The REAL bare-ball rung, on a synthetic board: a 12-ball part U1 whose
     ball 1 (SIG) has no copper, SIG's other pad inside a locked foreign ring
     so it stays open, C1 beside the escape site. The tap is made to fail and
     generate_bga_fanout returns, permissively, a dogbone whose via lands on
     C1; _cap_conflicts, _find_cap_relocation and the would-short guard are
     the real ones.
       a. declined (a foreign via beside the escape site): nothing is queued,
          C1 stays put on the CLI's board, the GUI posts nothing and its
          apply leaves C1.
       b. shipped: the escape via is on the CLI's board AND C1 is at its
          relocation; the GUI posts the same move, its change-set carries the
          via, and its apply moves C1 to the same place. SIG cannot close
          here by construction, so #1778's withdrawal of an escape that
          bought nothing is held off in this arm alone (both fronts).
       c. withdrawn (#1778, the real withdrawal): the escape bought nothing,
          so neither front ships its via or moves C1.

ON ipc-migration the Route tab is routing_dialog.py's (this branch's
swig_gui.py) and its live board is tests/gui_parity/fake_ipc_board: the
board FILE, every IPC commit flushed back into it (footprint moves through
placement.writer), so the GUI arms read C1's live pose off that file. The
stager is the IPC tab's own: a save_board_snapshot plus the board's
siblings. The apply is the IPC _apply_results_to_board_body -- the cap
moved through kicad_ipc_adapter.apply_footprint_moves, and the fallback
re-weld through apply_oracle_reconnect. pcbnew remains the fixture's
AUTHORING tool only (a filled pour, saved); the plugin never calls it.

Needs KiCad's python (pcbnew + wx + kipy); exits 2 when there is none -- it
does NOT self-skip, so it cannot report a gap it never looked at as closed.
"""
import contextlib
import glob
import io
import os
import shutil
import subprocess
import sys
import tempfile

REPO = os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

sys.path.insert(0, os.path.join(REPO, 'py_router'))
from kicad_locate import path_version_key  # noqa: E402
del sys.path[0]    # this file orders its own sys.path further down
KICAD_PYTHONS = [
    "/Applications/KiCad/KiCad.app/Contents/Frameworks/Python.framework/"
    "Versions/Current/bin/python3",
    "/usr/bin/python3",
    os.path.expandvars(r"C:\Program Files\KiCad\bin\python.exe"),
    *sorted(glob.glob(r"C:\Program Files\KiCad\*\bin\python.exe"),
            key=path_version_key, reverse=True),
]

BOARD = os.path.join(REPO, 'kicad_files', 'cap_chain.kicad_pcb')
CAP, OTHER = 'C1', 'C2'
DX = 0.7
POUR_NET, POUR_LAYER = 'DPA_N', 'B.Cu'
NETS = ['DPA_P', 'DPA_N']
SUBRUN_NET = 'DPA_P'
# Same kwargs on both fronts. A power width, a per-net class clearance and a
# .kicad_dru layer rule give every by-name map the re-weld carries a value, so
# comparing the two fronts' maps is not comparing two empty dicts. The scalars
# sit off the oracle helper's fallbacks (hole-to-hole 0.20, track-to-via 0.8),
# so an argument the applier dropped cannot match by coincidence.
KW = dict(layers=['F.Cu', 'B.Cu'], track_width=0.2, clearance=0.2,
          via_size=0.6, via_drill=0.3, grid_step=0.1,
          hole_to_hole_clearance=0.25, layer_costs=[1.0, 3.0],
          power_nets=['DPB_P'], power_nets_widths=[0.4])
CLASS_NET, CLASS_CLEARANCE = 'DPB_P', 0.25
DRU = ('(version 1)\n(rule "b_tight" (layer "B.Cu") '
       '(constraint clearance (min 0.25mm)))\n')
CFG_FIELDS = ('clearance', 'track_width', 'via_size', 'via_drill', 'grid_step',
              'layers', 'layer_costs', 'layer_clearances')
KW_FIELDS = ('track_via_clearance', 'hole_to_hole_clearance',
             'net_clearances_by_name', 'net_widths_by_name')
APPLY_CFG = {'move_copper_text': False, 'fix_drc_settings': False}

# ---- arm 6: the synthetic 12-ball board ------------------------------------
ESC_PAD = (8.5, 9.0)          # U1.1 (SIG), a bare ball
ESC_VIA = (8.5, 7.6)          # the permissive dogbone's via
C1_AT = (7.32, 7.6)           # its right pad 0.70 mm from the via: a real
#                               conflict at this pose, clear once relocated
BLOCKER_VIA = (9.25, 7.6)     # arm 6a: a foreign via beside the escape site
RUNG_KW = dict(layers=['F.Cu', 'B.Cu'], track_width=0.2, clearance=0.2,
               via_size=0.6, via_drill=0.3, grid_step=0.1,
               hole_to_hole_clearance=0.25)


def _reexec_into_kicad():
    for cand in KICAD_PYTHONS:
        if cand == sys.executable or not os.path.exists(cand):
            continue
        if subprocess.run([cand, '-c', 'import pcbnew, wx, kipy'],
                          capture_output=True).returncode == 0:
            # os.execv re-splits argv on spaces on Windows ("Program Files").
            sys.exit(subprocess.run([cand, os.path.abspath(__file__)]
                                    + sys.argv[1:]).returncode)
    print("ERROR: no python with pcbnew+wx+kipy found; this gate does not "
          "self-skip")
    sys.exit(2)


FAILURES = []


def check(name, cond, info=""):
    if not cond:
        FAILURES.append(name)
    print(("  PASS " if cond else "  FAIL ") + name + (f"  ({info})" if info else ""))


class _Patched:
    """Swap one module attribute for the duration of a `with`."""

    def __init__(self, module, name, value):
        self.module, self.name, self.value = module, name, value

    def __enter__(self):
        self.orig = getattr(self.module, self.name)
        setattr(self.module, self.name, self.value)
        return self

    def __exit__(self, *exc):
        setattr(self.module, self.name, self.orig)
        return False


class _Env:
    """Set environment variables for the duration of a `with`."""

    def __init__(self, **kv):
        self.kv = kv

    def __enter__(self):
        self.old = {k: os.environ.get(k) for k in self.kv}
        os.environ.update(self.kv)

    def __exit__(self, *exc):
        for k, v in self.old.items():
            if v is None:
                os.environ.pop(k, None)
            else:
                os.environ[k] = v
        return False


def _stage_fixture(d):
    """cap_chain + a pcbnew-authored B.Cu pour on DPA_N, saved with pcbnew."""
    import pcbnew
    from kicad_parser import mm_to_iu
    src = os.path.join(d, 'src.kicad_pcb')
    shutil.copyfile(BOARD, src)
    b = pcbnew.LoadBoard(src)
    z = pcbnew.ZONE(b)
    z.SetNet(b.FindNet(POUR_NET))
    z.SetLayer(b.GetLayerID(POUR_LAYER))
    ol = z.Outline()
    ol.NewOutline()
    for x, y in ((6, 6), (30, 6), (30, 24), (6, 24)):
        ol.Append(mm_to_iu(x), mm_to_iu(y))
    b.Add(z)
    pcbnew.ZONE_FILLER(b).Fill(b.Zones())
    out = os.path.join(d, 'fixture.kicad_pcb')
    pcbnew.SaveBoard(out, b)
    with open(os.path.splitext(out)[0] + '.kicad_dru', 'w') as f:
        f.write(DRU)
    return out


def _rung_board(blocker):
    """Arm 6's board (KiCad 8 text, read by both parse paths and pcbnew)."""
    pads, n = [], 1
    for row in range(3):
        for col in range(4):
            net = ' (net 1 "SIG")' if n == 1 else ''
            pads.append(f'    (pad "{n}" smd circle (at {-1.5 + col} {-1 + row})'
                        f' (size 0.4 0.4) (layers "F.Cu" "F.Paste" "F.Mask"){net})\n')
            n += 1
    ring = [(23.5, 8.5), (26.5, 8.5), (26.5, 11.5), (23.5, 11.5), (23.5, 8.5)]
    segs = ''.join(
        f'  (segment (start {x1} {y1}) (end {x2} {y2}) (width 0.3) '
        f'(layer "{layer}") (locked yes) (net 4))\n'
        for layer in ('F.Cu', 'B.Cu')
        for (x1, y1), (x2, y2) in zip(ring, ring[1:]))
    blk = (f'  (via (at {BLOCKER_VIA[0]} {BLOCKER_VIA[1]}) (size 0.6) (drill 0.3)'
           f' (layers "F.Cu" "B.Cu") (net 4))\n' if blocker else '')
    return ('(kicad_pcb (version 20240108) (generator "test")\n'
            '  (general (thickness 1.6))\n'
            '  (layers (0 "F.Cu" signal) (31 "B.Cu" signal) (44 "Edge.Cuts" user))\n'
            '  (setup (pad_to_mask_clearance 0))\n'
            '  (net 0 "")\n  (net 1 "SIG")\n  (net 2 "CA")\n  (net 3 "CB")\n'
            '  (net 4 "BLK")\n'
            '  (footprint "t:GRID12" (layer "F.Cu") (at 10 10)\n'
            '    (property "Reference" "U1" (at 0 0) (layer "F.SilkS"))\n'
            + ''.join(pads) + '  )\n'
            f'  (footprint "t:C0402" (layer "F.Cu") (at {C1_AT[0]} {C1_AT[1]})\n'
            '    (property "Reference" "C1" (at 0 0) (layer "F.SilkS"))\n'
            '    (pad "1" smd rect (at -0.48 0) (size 0.56 0.62)'
            ' (layers "F.Cu" "F.Paste" "F.Mask") (net 2 "CA"))\n'
            '    (pad "2" smd rect (at 0.48 0) (size 0.56 0.62)'
            ' (layers "F.Cu" "F.Paste" "F.Mask") (net 3 "CB")))\n'
            '  (footprint "t:TH" (layer "F.Cu") (at 25 10)\n'
            '    (property "Reference" "J1" (at 0 0) (layer "F.SilkS"))\n'
            '    (pad "1" thru_hole circle (at 0 0) (size 1.7 1.7) (drill 1.0)'
            ' (layers "*.Cu" "*.Mask") (net 1 "SIG")))\n'
            + segs + blk +
            '  (gr_rect (start 0 0) (end 30 20) (stroke (width 0.1) (type default))'
            ' (fill none) (layer "Edge.Cuts"))\n)\n')


def _class_map(nets):
    """{net id: clearance} for CLASS_NET on THIS front's net numbering."""
    return {nid: CLASS_CLEARANCE for nid, n in nets.items()
            if n.name == CLASS_NET}


def _fresh(d, src, tag):
    """A fresh copy (+ siblings) per arm: one path per pcbnew load."""
    sub = os.path.join(d, tag)
    os.makedirs(sub)
    dst = os.path.join(sub, 'b.kicad_pcb')
    stem = os.path.splitext(src)[0]
    for ext in ('.kicad_pcb', '.kicad_pro', '.kicad_prl', '.kicad_dru'):
        if os.path.exists(stem + ext):
            shutil.copyfile(stem + ext, os.path.splitext(dst)[0] + ext)
    return dst


def _norm(v):
    if isinstance(v, dict):
        return {k: _norm(x) for k, x in sorted(v.items())}
    if isinstance(v, (list, tuple)):
        return [_norm(x) for x in v]
    if isinstance(v, float):
        return round(v, 9)
    return v


def _near(a, b, tol=1e-6):
    return (a is not None and b is not None and not isinstance(a, str)
            and abs(a[0] - b[0]) < tol and abs(a[1] - b[1]) < tol)


def _quiet(fn):
    """Run fn with its stdout captured; return (result, its #666 lines)."""
    buf = io.StringIO()
    with contextlib.redirect_stdout(buf):
        out = fn()
    lines = [ln.strip() for ln in buf.getvalue().splitlines()
             if '#666' in ln or 'bare-ball escape' in ln]
    return out, lines


def _show(lines):
    for ln in lines:
        print(f"     | {ln[:150]}")


class _Arm:
    """One arm's rescue wrapper, oracle spy and #984 bridge spy."""

    def __init__(self, route, model=None, queue_in=None, weld_nets=()):
        self.route = route
        self.model = model          # the GUI's PCBData, read at each oracle call
        self.queue_in = queue_in    # None | 'main' | 'finalize'
        self.weld_nets = sorted(weld_nets)
        self.queued = None
        self.queued_in_finalize = None
        self.calls = []
        self.bridged = []
        self.phase = 'engine'

    def rescue(self, real):
        def _wrapped(state, *a, **kw):
            out = real(state, *a, **kw)
            in_fin = self.route._plane_finalize_active()
            if self.queue_in is None or self.queued is not None:
                return out
            if (self.queue_in == 'finalize') != in_fin:
                return out
            pcb = state.pcb_data
            fp = pcb.footprints[CAP]
            ids = sorted({p.net_id for p in fp.pads if p.net_id})
            self.queued = {
                'reference': CAP, 'new_x': fp.x + DX, 'new_y': fp.y,
                'new_rotation': fp.rotation, 'net_ids': ids,
                'net_names': sorted(pcb.nets[i].name for i in ids),
                'from': (fp.x, fp.y)}
            self.queued_in_finalize = in_fin
            q = getattr(pcb, '_pending_cap_moves', None)
            if q is None:
                q = pcb._pending_cap_moves = []
            q.append({k: v for k, v in self.queued.items() if k != 'from'})
            return out
        return _wrapped

    def oracle(self, board_file, net_names, config, *a, **kw):
        from kicad_parser import parse_kicad_pcb
        try:
            fp = parse_kicad_pcb(board_file).footprints.get(CAP)
            where = (fp.x, fp.y) if fp is not None else None
        except Exception as e:                                  # noqa: BLE001
            where = f"unparsed: {e}"
        mfp = (self.model.footprints.get(CAP)
               if self.model is not None else None)
        nets = sorted(net_names or [])
        self.calls.append({
            'nets': nets, 'phase': self.phase,
            'in_finalize': self.route._plane_finalize_active(),
            'cfg': {f: _norm(getattr(config, f, None)) for f in CFG_FIELDS},
            'kw': {f: _norm(kw.get(f)) for f in KW_FIELDS},
            'cap_at': where,
            'model_cap_at': (mfp.x, mfp.y) if mfp is not None else None})
        welded = 1 if nets == self.weld_nets else 0
        return {'available': True, 'rounds': 1, 'links_routed': welded,
                'links_failed': 0, 'remaining': 0, 'new_segments': [],
                'new_vias': [], 'removed_segments': [], 'removed_vias': []}

    def bridge(self, real):
        def _wrapped(pcb_data, output_file, return_results, results_data,
                     write_model, scope_names, config):
            self.bridged.append(sorted(scope_names))
            return real(pcb_data, output_file, return_results, results_data,
                        write_model, scope_names, config)
        return _wrapped

    def welds(self, nets, phase=None):
        return [c for c in self.calls if c['nets'] == sorted(nets)
                and (phase is None or c['phase'] == phase)]

    def bridged_nets(self):
        return sorted({n for b in self.bridged for n in b})


def _patches(arm, mods, rescue=True):
    st = contextlib.ExitStack()
    if rescue:
        st.enter_context(_Patched(mods['net_rescue'], 'rescue_failed_nets',
                                  arm.rescue(mods['real_rescue'])))
    st.enter_context(_Patched(mods['kicad_oracle'], 'oracle_reconnect',
                              arm.oracle))
    st.enter_context(_Patched(mods['route'], '_late_soft_joint_bridge984',
                              arm.bridge(mods['real_bridge'])))
    return st


def _repair_with_subrun(mods, kw):
    """The finalize's repair, preceded by one real nested batch_route sub-run
    on the board the finalize handed it -- as repair_planes' own reconnects
    run (return_results, the shared pcb_data, final_reconcile=False). Without
    live reuse the finalize hands it no board: like the real repair it then
    works on a parse of its own."""
    real = mods['real_repair']
    from kicad_parser import parse_kicad_pcb

    def _wrapped(*a, **k):
        board = k.get('pcb_data')
        if board is None:
            board = k['pcb_data'] = parse_kicad_pcb(a[0])
        mods['route'].batch_route(
            a[0] or "", "", [SUBRUN_NET], return_results=True,
            pcb_data=board, final_reconcile=False, **kw)
        return real(*a, **k)
    return _wrapped


def _gui_setup(path, swig_gui):
    from fake_ipc_board import install, build_pcb_data_like_ipc
    live = install(path)
    dialog = swig_gui.RoutingDialog(None, build_pcb_data_like_ipc(path), path)
    dialog._suppress_completion_popups = True
    return live, dialog


def _stager(live, d):
    def _stage():
        # The IPC tab's _stage_live_board: a snapshot of the LIVE board (no
        # project) and the board's siblings beside it (#627).
        from copy_board import SIBLING_EXTS
        from kicad_ipc_adapter import save_board_snapshot
        with tempfile.NamedTemporaryFile(suffix='.kicad_pcb', dir=d,
                                         delete=False) as f:
            p = f.name
        save_board_snapshot(p)
        for ext in SIBLING_EXTS:
            sib = os.path.splitext(live.path)[0] + ext
            if os.path.isfile(sib):
                shutil.copy2(sib, os.path.splitext(p)[0] + ext)
        return p
    return _stage


def _live_fp(live, ref):
    from kicad_parser import parse_kicad_pcb
    return parse_kicad_pcb(live.path).footprints.get(ref)


def _live_xy(live, ref):
    fp = _live_fp(live, ref)
    return (fp.x, fp.y) if fp is not None else None


def _live_pads(live, ref):
    fp = _live_fp(live, ref)
    return [(p.global_x, p.global_y) for p in fp.pads] if fp is not None else []


def _apply(dialog, rd, ok, failed, arm, mods):
    arm.phase = 'apply'
    with _patches(arm, mods, rescue=False), \
            _Patched(mods['kicad_oracle'], 'find_kicad_cli',
                     lambda *a, **k: '/kicad-cli/stand-in'):
        _, lines = _quiet(lambda: dialog._apply_results_to_board_body(
            rd, ok, failed, 0.0, dict(APPLY_CFG)))
    arm.phase = 'engine'
    return lines


# --------------------------------------------------------------------- arms

def arms_1_to_4(d, fixture, mods, swig_gui):
    import pcbnew
    from kicad_parser import parse_kicad_pcb
    route = mods['route']
    batch_route = route.batch_route

    print("-- 1. CLI, a move the main pass queued (the reference) --")
    cli_in = _fresh(d, fixture, 'cli')
    cli_out = os.path.join(os.path.dirname(cli_in), 'out.kicad_pcb')
    cap_nets = sorted(parse_kicad_pcb(cli_in).nets[n].name
                      for n in {p.net_id for p in
                                parse_kicad_pcb(cli_in).footprints[CAP].pads})
    cli = _Arm(route, queue_in='main', weld_nets=cap_nets)
    with _patches(cli, mods):
        _, lines = _quiet(lambda: batch_route(
            cli_in, cli_out, list(NETS),
            net_clearances=_class_map(parse_kicad_pcb(cli_in).nets), **KW))
    _show(lines)
    check("the wrapped rescue queued the move (CLI)", cli.queued is not None)
    if cli.queued is None:
        return None
    target = (cli.queued['new_x'], cli.queued['new_y'])
    check("the target differs from C1's pose",
          not _near(target, cli.queued['from']),
          f"{cli.queued['from']} -> {target}")
    written = parse_kicad_pcb(cli_out).footprints
    check("the written board has C1 at the target",
          _near((written[CAP].x, written[CAP].y), target),
          f"({written[CAP].x}, {written[CAP].y})")
    cli_weld = cli.welds(cap_nets)
    check("the CLI re-welded C1's nets once", len(cli_weld) == 1,
          f"{len(cli_weld)} on {cap_nets}; all: {[c['nets'] for c in cli.calls]}")
    order = [c['nets'] for c in cli.calls]
    check("... before the plane finalize's oracle leg (non-vacuity: it ran)",
          cap_nets in order and [POUR_NET] in order
          and order.index(cap_nets) < order.index([POUR_NET]), f"{order}")
    if cli_weld:
        r0 = cli_weld[0]
        check("the CLI re-weld's by-name maps all carry a value (non-vacuity)",
              (r0['cfg']['layer_clearances'] or {}).get('B.Cu') == 0.25
              and (r0['kw']['net_clearances_by_name'] or {}).get(CLASS_NET)
              == CLASS_CLEARANCE
              and ((r0['kw']['net_widths_by_name'] or {}).get(
                  'power_net_widths') or {}).get('DPB_P') == 0.4,
              f"{r0['cfg']['layer_clearances']}, {r0['kw']}")
    check("every CLI oracle call saw C1 at the target",
          cli.calls and all(_near(c['cap_at'], target) for c in cli.calls),
          f"{[(c['nets'], c['cap_at']) for c in cli.calls]}")
    check("the CLI handed C1's nets to the #984 bridge",
          set(cap_nets) <= set(cli.bridged_nets()), f"{cli.bridged}")
    ref = {'cfg': cli_weld[0]['cfg'], 'kw': cli_weld[0]['kw']} if cli_weld else None

    print("-- 2. GUI engine, with the tab's stage_board_fn --")
    gui_path = _fresh(d, fixture, 'gui')
    live, dialog = _gui_setup(gui_path, swig_gui)
    pcb = dialog.pcb_data
    gui = _Arm(route, model=pcb, queue_in='main', weld_nets=cap_nets)
    c1_pads_before = [(p.global_x, p.global_y) for p in pcb.footprints[CAP].pads]
    live_c1_pads = _live_pads(live, CAP)
    live_c2 = _live_xy(live, OTHER)
    try:
        with _patches(gui, mods):
            (ok, failed, _t, rd), lines = _quiet(lambda: batch_route(
                gui_path, "", list(NETS), return_results=True, pcb_data=pcb,
                stage_board_fn=_stager(live, d),
                net_clearances=_class_map(pcb.nets), **KW))
        _show(lines)
        check("the wrapped rescue queued the same target (GUI)",
              gui.queued is not None
              and _near((gui.queued['new_x'], gui.queued['new_y']), target))
        weld = gui.welds(cap_nets, 'engine')
        check("the re-weld ran IN-RUN, once", len(weld) == 1,
              f"{len(weld)}; all: {[(c['nets'], c['phase']) for c in gui.calls]}")
        order = [c['nets'] for c in gui.calls]
        check("... in the CLI's order (before the finalize's oracle leg)",
              cap_nets in order and [POUR_NET] in order
              and order.index(cap_nets) < order.index([POUR_NET]), f"{order}")
        if weld and ref:
            check("... with the CLI's re-weld config", weld[0]['cfg'] == ref['cfg'],
                  f"GUI {weld[0]['cfg']} vs CLI {ref['cfg']}")
            check("... and the CLI's re-weld kwargs", weld[0]['kw'] == ref['kw'],
                  f"GUI {weld[0]['kw']} vs CLI {ref['kw']}")
            check("... on a staged board where C1 sits at the target",
                  _near(weld[0]['cap_at'], target), f"{weld[0]['cap_at']}")
            check("... and a model where it does too",
                  _near(weld[0]['model_cap_at'], target),
                  f"{weld[0]['model_cap_at']}")
        pour = [c for c in gui.calls if c['nets'] == [POUR_NET]]
        check("the finalize's staged oracle board has C1 at the target, as the "
              "CLI's written board does",
              pour and all(_near(c['cap_at'], target) for c in pour),
              f"{[c['cap_at'] for c in pour]}")
        check("the GUI handed C1's nets to the #984 bridge, as the CLI does",
              set(cap_nets) <= set(gui.bridged_nets()), f"{gui.bridged}")
        spec = rd.get('pending_cap_moves') or {}
        moves = spec.get('moves') or []
        check("results_data['pending_cap_moves'] carries the move, re-welded "
              "'in-run'",
              len(moves) == 1 and moves[0].get('reference') == CAP
              and _near((moves[0].get('new_x'), moves[0].get('new_y')), target)
              and moves[0].get('reweld') == 'in-run', f"{moves}")
        fp1 = pcb.footprints[CAP]
        check("the model is mirrored, pads included",
              _near((fp1.x, fp1.y), target)
              and all(_near((p.global_x, p.global_y), (bx + DX, by))
                      for p, (bx, by) in zip(fp1.pads, c1_pads_before)),
              f"({fp1.x}, {fp1.y})")

        print("-- 3. GUI apply of arm 2 (the real _apply_results_to_board_body) --")
        gui.calls = []
        _show(_apply(dialog, rd, ok, failed, gui, mods))
        check("the live board has C1 at the target",
              _near(_live_xy(live, CAP), target, 1e-5), f"{_live_xy(live, CAP)}")
        check("... with its pads moved by the offset",
              all(_near(a, (bx + DX, by), 1e-5)
                  for a, (bx, by) in zip(_live_pads(live, CAP), live_c1_pads)),
              f"{_live_pads(live, CAP)} from {live_c1_pads}")
        check("C2 is untouched on the live board",
              _near(_live_xy(live, OTHER), live_c2, 1e-9))
        check("the apply does NOT re-weld again (the engine did, in-run)",
              not gui.welds(cap_nets), f"{[c['nets'] for c in gui.calls]}")
        fpd = dialog.pcb_data.footprints[CAP]
        check("the dialog's pcb_data agrees",
              _near((fpd.x, fpd.y), target), f"({fpd.x}, {fpd.y})")
    finally:
        dialog.Destroy()

    print("-- 4. GUI fallback: no stage_board_fn --")
    fb_path = _fresh(d, fixture, 'gui_fallback')
    live, dialog = _gui_setup(fb_path, swig_gui)
    pcb = dialog.pcb_data
    fb = _Arm(route, model=pcb, queue_in='main', weld_nets=cap_nets)
    try:
        with _patches(fb, mods):
            (ok, failed, _t, rd), lines = _quiet(lambda: batch_route(
                fb_path, "", list(NETS), return_results=True, pcb_data=pcb,
                net_clearances=_class_map(pcb.nets), **KW))
        _show(lines)
        check("no in-run re-weld without a staged board",
              not fb.welds(cap_nets), f"{[c['nets'] for c in fb.calls]}")
        check("... and nothing of C1's for the #984 bridge",
              not (set(cap_nets) & set(fb.bridged_nets())), f"{fb.bridged}")
        moves = (rd.get('pending_cap_moves') or {}).get('moves') or []
        check("the move is posted, marked for the applier's re-weld",
              len(moves) == 1 and moves[0].get('reweld') == 'applier', f"{moves}")
        _show(_apply(dialog, rd, ok, failed, fb, mods))
        check("the apply moves C1 to the target",
              _near(_live_xy(live, CAP), target, 1e-5), f"{_live_xy(live, CAP)}")
        weld = fb.welds(cap_nets, 'apply')
        check("... and re-welds once, after apply", len(weld) == 1,
              f"{[(c['nets'], c['phase']) for c in fb.calls]}")
        if weld and ref:
            check("... with the CLI's re-weld config and kwargs",
                  weld[0]['cfg'] == ref['cfg'] and weld[0]['kw'] == ref['kw'],
                  f"GUI {weld[0]['cfg']} {weld[0]['kw']} vs CLI {ref}")
            check("... on a board where C1 already sits at the target",
                  _near(weld[0]['cap_at'], target), f"{weld[0]['cap_at']}")
    finally:
        dialog.Destroy()
    return target, cap_nets


def arm_5(d, fixture, mods, swig_gui):
    import pcbnew
    from kicad_parser import parse_kicad_pcb
    route = mods['route']
    batch_route = route.batch_route
    sub_kw = {k: v for k, v in KW.items()}
    print("-- 5. a move queued INSIDE the plane finalize (a nested sub-run) --")
    for live_reuse in ('1', '0'):
        tag = 'with' if live_reuse == '1' else 'without'
        src = _fresh(d, fixture, f'fin_cli_{live_reuse}')
        out = os.path.join(os.path.dirname(src), 'out.kicad_pcb')
        cap_nets = sorted(parse_kicad_pcb(src).nets[n].name
                          for n in {p.net_id for p in
                                    parse_kicad_pcb(src).footprints[CAP].pads})
        arm = _Arm(route, queue_in='finalize', weld_nets=cap_nets)
        with _patches(arm, mods), \
                _Patched(mods['repair_planes'], 'repair_planes',
                         _repair_with_subrun(mods, sub_kw)), \
                _Env(KICAD_FINALIZE_ONLY='1', KICAD_PLANE_FINALIZE_LIVE=live_reuse):
            _, lines = _quiet(lambda: batch_route(
                src, out, list(NETS), skip_routing=True,
                net_clearances=_class_map(parse_kicad_pcb(src).nets), **KW))
        _show(lines)
        check(f"CLI ({tag} live reuse): the move was queued inside the "
              f"finalize (non-vacuity)", arm.queued_in_finalize is True,
              f"{arm.queued_in_finalize}")
        if not arm.queued:
            continue
        target = (arm.queued['new_x'], arm.queued['new_y'])
        w = parse_kicad_pcb(out).footprints[CAP]
        check(f"CLI ({tag} live reuse): the written board has C1 at the target",
              _near((w.x, w.y), target), f"({w.x}, {w.y}) vs {target}")
        weld = arm.welds(cap_nets)
        check(f"CLI ({tag} live reuse): ... re-welded once, after the finalize",
              len(weld) == 1 and not weld[0]['in_finalize'],
              f"{[(c['nets'], c['in_finalize']) for c in arm.calls]}")
        pour = [c for c in arm.calls if c['nets'] == [POUR_NET]]
        check(f"CLI ({tag} live reuse): the finalize's oracle leg ran with C1 "
              f"where it was (the move comes after)",
              pour and all(_near(c['cap_at'], arm.queued['from'])
                           for c in pour), f"{[c['cap_at'] for c in pour]}")

    src = _fresh(d, fixture, 'fin_gui')
    live, dialog = _gui_setup(src, swig_gui)
    pcb = dialog.pcb_data
    cap_nets = sorted(pcb.nets[n].name
                      for n in {p.net_id for p in pcb.footprints[CAP].pads})
    arm = _Arm(route, model=pcb, queue_in='finalize', weld_nets=cap_nets)
    try:
        with _patches(arm, mods), \
                _Patched(mods['repair_planes'], 'repair_planes',
                         _repair_with_subrun(mods, sub_kw)), \
                _Env(KICAD_FINALIZE_ONLY='1'):
            (ok, failed, _t, rd), lines = _quiet(lambda: batch_route(
                src, "", list(NETS), return_results=True, pcb_data=pcb,
                stage_board_fn=_stager(live, d), skip_routing=True,
                net_clearances=_class_map(pcb.nets), **KW))
        _show(lines)
        check("GUI: the move was queued inside the finalize (non-vacuity)",
              arm.queued_in_finalize is True, f"{arm.queued_in_finalize}")
        if arm.queued:
            target = (arm.queued['new_x'], arm.queued['new_y'])
            weld = arm.welds(cap_nets, 'engine')
            check("GUI: ... re-welded in-run once, after the finalize",
                  len(weld) == 1 and not weld[0]['in_finalize'],
                  f"{[(c['nets'], c['phase'], c['in_finalize']) for c in arm.calls]}")
            pour = [c for c in arm.calls if c['nets'] == [POUR_NET]]
            check("GUI: the finalize's staged oracle board had C1 where it "
                  "was, as the CLI's file does",
                  pour and all(_near(c['cap_at'], arm.queued['from'])
                               for c in pour), f"{[c['cap_at'] for c in pour]}")
            moves = (rd.get('pending_cap_moves') or {}).get('moves') or []
            check("GUI: ... and posted for the applier",
                  len(moves) == 1 and _near((moves[0]['new_x'],
                                             moves[0]['new_y']), target),
                  f"{moves}")
            _show(_apply(dialog, rd, ok, failed, arm, mods))
            check("GUI: the apply moves C1 to the target",
                  _near(_live_xy(live, CAP), target, 1e-5),
                  f"{_live_xy(live, CAP)} vs {target}")
    finally:
        dialog.Destroy()


def arm_6(d, mods, swig_gui):
    import pcbnew
    from kicad_parser import parse_kicad_pcb
    import bga_fanout
    import plane_pad_tap
    route = mods['route']
    batch_route = route.batch_route

    def fanout(fp, pcb, net_filter=None, **kw):
        # strict attempt (every passive immovable): no escape; permissive:
        # a dogbone from the bare ball to a via that lands on C1
        if getattr(pcb, '_fanout_all_foreign_immovable', False):
            return [], []
        sig = next(i for i, n in pcb.nets.items() if n.name == 'SIG')
        return ([{'start': ESC_PAD, 'end': ESC_VIA, 'width': 0.2,
                  'layer': 'F.Cu', 'net_id': sig}],
                [{'x': ESC_VIA[0], 'y': ESC_VIA[1], 'size': 0.6, 'drill': 0.3,
                  'layers': ['F.Cu', 'B.Cu'], 'net_id': sig}])

    def via_at_site(vias, nets):
        return [v for v in vias if nets.get(v.net_id)
                and nets[v.net_id].name == 'SIG'
                and ((v.x - ESC_VIA[0]) ** 2 + (v.y - ESC_VIA[1]) ** 2) ** 0.5
                <= 0.3]

    print("-- 6. the real bare-ball rung on a synthetic 12-ball part --")
    results = {}
    def hold_1778(real):
        # the terrain-only withdrawal (#1778) passes the escapes back as they
        # are; the whole-net one (with_traces False) runs as it is
        def wrapped(pcb_data, net_id, tap_results, config, with_traces=False):
            if with_traces:
                return tap_results
            return real(pcb_data, net_id, tap_results, config,
                        with_traces=with_traces)
        return wrapped

    for case, blocker, hold in (('a. declined', True, False),
                                ('b. shipped', False, True),
                                ('c. withdrawn', False, False)):
        sub = os.path.join(d, 'rung_' + case[0])
        os.makedirs(sub)
        src = os.path.join(sub, 'b.kicad_pcb')
        with open(src, 'w') as f:
            f.write(_rung_board(blocker))
        out = os.path.join(sub, 'out.kicad_pcb')
        arm = _Arm(route, queue_in=None, weld_nets=['CA', 'CB'])
        queued = []
        real_rescue = mods['real_rescue']

        def watch(state, *a, **kw):
            o = real_rescue(state, *a, **kw)
            queued.extend(getattr(state.pcb_data, '_pending_cap_moves', None)
                          or [])
            return o
        nr = mods['net_rescue']
        with contextlib.ExitStack() as held:
            if hold:
                held.enter_context(_Patched(
                    nr, '_withdraw_unused_escapes',
                    hold_1778(nr._withdraw_unused_escapes)))
            with _patches(arm, mods, rescue=False), \
                    _Patched(nr, 'rescue_failed_nets', watch), \
                    _Patched(plane_pad_tap, 'tap_pad_with_escalation',
                             lambda *a, **k: None), \
                    _Patched(bga_fanout, 'generate_bga_fanout', fanout):
                _, lines = _quiet(lambda: batch_route(src, out, ['SIG'], **RUNG_KW))
            print(f"   {case} -- CLI")
            _show(lines)
            reached = any('relocation to' in ln for ln in lines)
            check(f"6{case[0]} CLI: the rung reached the permissive retry and "
                  f"verified a relocation (non-vacuity)", reached)
            w = parse_kicad_pcb(out)
            c1 = (w.footprints[CAP].x, w.footprints[CAP].y)
            site = via_at_site(w.vias, w.nets)
            gui_out = _rung_gui(src, d, swig_gui, mods, fanout, plane_pad_tap,
                                bga_fanout)
        results[case] = (c1, site, queued, gui_out)
        if case[0] == 'c':
            check("6c CLI: the escape that bought nothing ships no via",
                  not site, f"{[(v.x, v.y) for v in site]}")
            check("6c CLI: C1 stays put", _near(c1, C1_AT), f"{c1}")
            check("6c GUI: no move posted",
                  not gui_out['posted'], f"{gui_out['posted']}")
            check("6c GUI: the apply leaves C1 where it was",
                  _near(gui_out['live_c1'], C1_AT, 1e-5),
                  f"{gui_out['live_c1']}")
        elif blocker:
            check("6a CLI: the declined escape queued no move",
                  not queued, f"{queued}")
            check("6a CLI: C1 stays put on the written board",
                  _near(c1, C1_AT), f"{c1}")
            check("6a CLI: no escape via at the site", not site,
                  f"{[(v.x, v.y) for v in site]}")
            check("6a GUI: no move posted",
                  not gui_out['posted'], f"{gui_out['posted']}")
            check("6a GUI: the apply leaves C1 where it was",
                  _near(gui_out['live_c1'], C1_AT, 1e-5),
                  f"{gui_out['live_c1']}")
        else:
            check("6b CLI: the escape via ships", bool(site),
                  f"{[(v.x, v.y) for v in w.vias]}")
            check("6b CLI: C1 is moved off its pose", not _near(c1, C1_AT),
                  f"{c1}")
            posted = gui_out['posted']
            check("6b GUI: the same move is posted",
                  len(posted) == 1 and _near((posted[0]['new_x'],
                                              posted[0]['new_y']), c1, 1e-5),
                  f"{posted} vs CLI {c1}")
            check("6b GUI: its change-set carries the escape via",
                  gui_out['via_in_changeset'], f"{gui_out['via_in_changeset']}")
            check("6b GUI: the apply moves C1 where the CLI did",
                  _near(gui_out['live_c1'], c1, 1e-5),
                  f"{gui_out['live_c1']} vs CLI {c1}")
    return results


def _rung_gui(src, d, swig_gui, mods, fanout, plane_pad_tap, bga_fanout):
    import pcbnew
    route = mods['route']
    sub = os.path.join(os.path.dirname(src), 'gui')
    os.makedirs(sub)
    path = os.path.join(sub, 'b.kicad_pcb')
    shutil.copyfile(src, path)
    live, dialog = _gui_setup(path, swig_gui)
    pcb = dialog.pcb_data
    arm = _Arm(route, model=pcb, queue_in=None, weld_nets=['CA', 'CB'])
    try:
        with _patches(arm, mods, rescue=False), \
                _Patched(plane_pad_tap, 'tap_pad_with_escalation',
                         lambda *a, **k: None), \
                _Patched(bga_fanout, 'generate_bga_fanout', fanout):
            (ok, failed, _t, rd), lines = _quiet(lambda: route.batch_route(
                path, "", ['SIG'], return_results=True, pcb_data=pcb,
                stage_board_fn=_stager(live, d), **RUNG_KW))
        posted = (rd.get('pending_cap_moves') or {}).get('moves') or []
        sig = {i for i, n in pcb.nets.items() if n.name == 'SIG'}
        cs_vias = [v for r in rd.get('results', [])
                   for v in (r.get('new_vias') or []) if v.net_id in sig]
        near = [v for v in cs_vias
                if ((v.x - ESC_VIA[0]) ** 2 + (v.y - ESC_VIA[1]) ** 2) ** 0.5
                <= 0.3]
        _apply(dialog, rd, ok, failed, arm, mods)
        return {'posted': posted, 'via_in_changeset': bool(near),
                'live_c1': _live_xy(live, CAP)}
    finally:
        dialog.Destroy()


def main():
    try:
        import wx  # noqa: F401
        import pcbnew  # noqa: F401
        import kipy  # noqa: F401
    except ImportError:
        _reexec_into_kicad()

    os.environ.setdefault('WXSUPPRESS_SIZER_FLAGS_CHECK', '1')
    import wx
    sys.path.insert(0, REPO)
    for sub in ('py_router', 'py_placer', 'py_tools'):
        sys.path.insert(0, os.path.join(REPO, sub))
    sys.path.insert(0, os.path.dirname(REPO))
    sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

    app = wx.App(False)  # noqa: F841 - must outlive the dialogs
    wx.Log.SetActiveTarget(wx.LogStderr())
    wx.MessageBox = lambda *a, **k: wx.OK
    import kicad_oracle
    import net_rescue
    import repair_planes
    import route
    # routing_dialog.py is this branch's swig_gui.py (renamed by the IPC port).
    from kicad_routing_plugin import routing_dialog as swig_gui
    mods = {'route': route, 'net_rescue': net_rescue,
            'kicad_oracle': kicad_oracle, 'repair_planes': repair_planes,
            'real_rescue': net_rescue.rescue_failed_nets,
            'real_bridge': route._late_soft_joint_bridge984,
            'real_repair': repair_planes.repair_planes}

    d = tempfile.mkdtemp(prefix='test666_')
    try:
        fixture = _stage_fixture(d)
        arms_1_to_4(d, fixture, mods, swig_gui)
        arm_5(d, fixture, mods, swig_gui)
        arm_6(d, mods, swig_gui)
    finally:
        shutil.rmtree(d, ignore_errors=True)
    print()
    if FAILURES:
        print(f"FAILED ({len(FAILURES)}): " + "; ".join(FAILURES))
        return 1
    print("ALL PASS -- both fronts move, re-weld and keep the #666 cap move "
          "the same way")
    return 0


if __name__ == '__main__':
    sys.exit(main())
