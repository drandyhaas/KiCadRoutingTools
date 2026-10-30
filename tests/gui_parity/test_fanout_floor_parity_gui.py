#!/usr/bin/env python3
"""The fanout tab writes the floors bga_fanout's and qfn_fanout's mains write.

    python3 tests/gui_parity/test_fanout_floor_parity_gui.py

(re-execs into KiCad's bundled python automatically, like its siblings, and
does NOT self-skip without it: exit 2)

THE GAP. The CLI fanouts finish with `fix_project_for_output(clearance,
track_width, via size and drill when the step may write via floors, the
non-Default classes kept -- or capped at --clearance-ceiling, #1529)` -- no
hole-to-hole, no edge clearance beyond the
fab pin, and the copper-to-hole floor lowered to the clearance with the rest.
The fanout tab finished with `update_live_drc_floors` over its own set: the
config's hole-to-hole and edge clearance, never the copper-to-hole floor, and
classes clamped only under the Min-Clearance ceiling. So after the same
fanout the live board and the CLI's project declared different floors, and
every later step read them.

WHAT THIS DRIVES. The REAL `FanoutTab._apply_fanout_results` of a headless
RoutingDialog on a REAL pcbnew board, for a BGA step with a track and a via, a
QFN step with a track only, and a QFN step with no copper, on a board whose
project declares its rules (flat_hierarchy, Default 0.2 / Wide 0.4) and on a
project-less one (splitflap_driver). The live board's copper is then saved,
the board's ORIGINAL project put beside it, and the CLI writeback run over it
with the keywords the CLI mains pass -- spelled out HERE, independently of the
shared decision, so the gate tests that decision rather than itself. The
floors each front then DECLARES must be the same: every KiCad rule, the repo's
via-drill floor, and each net class's clearance.
"""
import glob
import json
import os
import shutil
import subprocess
import sys
import tempfile

REPO = os.path.dirname(os.path.dirname(os.path.dirname(
    os.path.abspath(__file__))))

sys.path.insert(0, os.path.join(REPO, 'py_router'))
from kicad_locate import path_version_key  # noqa: E402
del sys.path[0]
KICAD_PYTHONS = [
    '/Applications/KiCad/KiCad.app/Contents/Frameworks/Python.framework/'
    'Versions/Current/bin/python3',
    '/usr/bin/python3',
    *sorted(glob.glob(r"C:\Program Files\KiCad\*\bin\python.exe"),
            key=path_version_key, reverse=True),
]
MM = 1e6
#: The tab's config. Its hole-to-hole and edge settings are what the old tab
#: wrote and the CLI fanouts never pass.
CFG = {'fix_drc_settings': True, 'clearance': 0.15, 'track_width': 0.15,
       'via_size': 0.3, 'via_drill': 0.2, 'clearance_ceiling': None,
       'class_ceiling': None,
       'hole_to_hole_clearance': 0.3, 'board_edge_clearance': 0.4}
RULES = ('min_clearance', 'min_track_width', 'min_connection', 'min_via_diameter',
         'min_through_hole_diameter', 'min_via_annular_width',
         'min_hole_clearance', 'min_hole_to_hole', 'min_copper_edge_clearance')
BOARDS = {'project': 'flat_hierarchy.kicad_pcb',
          'project-less': 'splitflap_driver.kicad_pcb'}


def cli_kwargs(kind, tracks, vias):
    """What bga_fanout's / qfn_fanout's main hands fix_project_for_output, read
    off their writeback code: None = QFN with no copper (nothing written)."""
    if kind == 'qfn' and not (tracks or vias):
        return None
    via = kind != 'qfn' or bool(vias)
    return {'clearance': CFG['clearance'], 'track_width': CFG['track_width'],
            'via_diameter': CFG['via_size'] if via else None,
            'via_drill': CFG['via_drill'] if via else None,
            # #1529: the classes kept without a ceiling, capped with one
            'clamp_nondefault_netclasses': CFG['class_ceiling'] is not None,
            'class_ceiling': CFG['class_ceiling']}


def _reexec_into_kicad():
    for cand in KICAD_PYTHONS:
        if cand == sys.executable or not os.path.exists(cand):
            continue
        if subprocess.run([cand, '-c', 'import pcbnew, wx'],
                          capture_output=True).returncode == 0:
            sys.exit(subprocess.run([cand, os.path.abspath(__file__)]
                                    + sys.argv[1:]).returncode)
    print("ERROR: no python with pcbnew + wx found; this gate does not "
          "self-skip, because a gate that exits 0 without running reports "
          "everything it guards as checked.")
    sys.exit(2)


def main():
    try:
        import wx  # noqa: F401
        import pcbnew  # noqa: F401
    except ImportError:
        _reexec_into_kicad()
    os.environ.setdefault('WXSUPPRESS_SIZER_FLAGS_CHECK', '1')
    import contextlib
    import io
    import wx
    sys.path.insert(0, REPO)
    for sub in ('py_router', 'py_placer', 'py_tools'):
        sys.path.insert(0, os.path.join(REPO, sub))
    sys.path.insert(0, os.path.dirname(REPO))
    app = wx.App(False)  # noqa: F841
    import pcbnew as _pcbnew
    import fix_kicad_drc_settings as F
    from kicad_parser import parse_kicad_pcb
    from kicad_routing_plugin.swig_gui import RoutingDialog

    failures, tmpdirs = [], []

    def check(name, cond, info=""):
        if not cond:
            failures.append(name)
        print(("  PASS " if cond else "  FAIL ") + name + (f"  {info}" if info else ""))

    def default_declared(board):
        """The live Default class is DECLARED: the project has one, or a
        writer set it (fix_kicad_drc_settings.live_declares_default_class --
        on code without it, the project alone decides)."""
        fn = getattr(F, 'live_declares_default_class', None)
        if fn is not None:
            return fn(board)
        pro = os.path.splitext(board.GetFileName())[0] + '.kicad_pro'
        if not os.path.isfile(pro):
            return False
        with open(pro, encoding='utf-8') as f:
            doc = json.load(f)
        return any(c.get('name') == 'Default' for c in
                   (doc.get('net_settings') or {}).get('classes') or [])

    def classes_live(board):
        ns = board.GetDesignSettings().m_NetSettings
        out = {}
        if default_declared(board):
            out['Default'] = round(ns.GetDefaultNetclass().GetClearance() / MM, 6)
        try:
            for name, nc in ns.GetNetclasses().items():
                if str(name) != 'Default':
                    out[str(name)] = round(nc.GetClearance() / MM, 6)
        except Exception:
            pass
        return out

    def gui_floors(board):
        rules = {k: round(v, 6) for k, v in (F.live_declared_rules(board, RULES)
                                             or {}).items()}
        vdr = F.live_declared_fab_floor_rules(board).get('min_via_drill')
        if vdr is not None:
            rules['min_via_drill'] = round(vdr, 6)
        return rules, classes_live(board)

    def cli_floors(pro):
        if not os.path.isfile(pro):
            return {}, {}
        with open(pro, encoding='utf-8') as f:
            doc = json.load(f)
        # The rules as the next CLI step reads them (a repo-private rule from
        # the records when the project's rules lack it), where available.
        _eff = getattr(F, '_effective_rules', None)
        r = (_eff(doc) if _eff else
             ((doc.get('board') or {}).get('design_settings') or {}).get('rules') or {})
        rules = {k: round(float(r[k]), 6) for k in RULES + ('min_via_drill',)
                 if k in r and r[k] is not None}
        cls = {c['name']: round(float(c['clearance']), 6)
               for c in (doc.get('net_settings') or {}).get('classes') or []
               if isinstance(c, dict) and c.get('clearance') is not None}
        return rules, cls

    def run_arm(board_kind, kind, tracks, vias):
        src = os.path.join(REPO, 'kicad_files', BOARDS[board_kind])
        src_pro = os.path.splitext(src)[0] + '.kicad_pro'
        d = tempfile.mkdtemp(prefix='fanoutfloor_')
        tmpdirs.append(d)
        gui = os.path.join(d, 'gui', 'b.kicad_pcb')
        cli = os.path.join(d, 'cli', 'b.kicad_pcb')
        os.makedirs(os.path.dirname(gui))
        os.makedirs(os.path.dirname(cli))
        shutil.copyfile(src, gui)
        if os.path.isfile(src_pro):
            shutil.copyfile(src_pro, os.path.splitext(gui)[0] + '.kicad_pro')
        live = _pcbnew.LoadBoard(gui)
        dlg = RoutingDialog(None, parse_kicad_pcb(gui), gui)
        dlg._suppress_completion_popups = True
        tab = dlg.fanout_tab
        tab.on_fanout_complete = None
        real = _pcbnew.GetBoard
        _pcbnew.GetBoard = lambda: live
        try:
            with contextlib.redirect_stdout(io.StringIO()):
                tab._apply_fanout_results(tracks, vias, failed_nets=[],
                                          fanout_config=dict(CFG),
                                          fanout_kind=kind)
        finally:
            _pcbnew.GetBoard = real
            dlg.Destroy()
        # The CLI writeback over the SAME copper and the ORIGINAL project --
        # and none of the GUI session's records, which the live board carries
        # on itself (tool_records) and the save would hand the CLI arm.
        _pcbnew.SaveBoard(cli, live, True)
        import re as _re
        with open(cli, encoding='utf-8') as f:
            txt = f.read()
        txt = _re.sub(r'(?m)^[ \t]*\(property "kicad_routing_tools" "(?:[^"\\]|\\.)*"\)\n',
                      '', txt)
        with open(cli, 'w', encoding='utf-8') as f:
            f.write(txt)
        if os.path.isfile(src_pro):
            shutil.copyfile(src_pro, os.path.splitext(cli)[0] + '.kicad_pro')
        kw = cli_kwargs(kind, tracks, vias)
        if kw is not None:
            with contextlib.redirect_stdout(io.StringIO()):
                F.fix_project_for_output(cli, input_pcb=src, **kw)
        return gui_floors(live), cli_floors(os.path.splitext(cli)[0] + '.kicad_pro')

    for board_kind in BOARDS:
        net = next(n for n in parse_kicad_pcb(
            os.path.join(REPO, 'kicad_files', BOARDS[board_kind])).nets if n)
        track = {'start': (100.0, 100.0), 'end': (101.0, 100.0), 'width': 0.15,
                 'layer': 'F.Cu', 'net_id': net}
        via = {'x': 101.0, 'y': 100.0, 'size': 0.3, 'drill': 0.2, 'net_id': net,
               'layers': ['F.Cu', 'B.Cu']}
        for label, kind, tracks, vias in (
                ('BGA, a track and a via', 'bga', [track], [via]),
                ('QFN, a track, no via', 'qfn', [track], []),
                ('QFN, no copper', 'qfn', [], [])):
            print(f"-- {board_kind} board, {label} --")
            (g_rules, g_cls), (c_rules, c_cls) = run_arm(board_kind, kind,
                                                         tracks, vias)
            if kind == 'qfn' and not tracks:
                # Nothing is written on either front; the live board keeps
                # what it declared, the CLI's project is the input's.
                src_pro = os.path.join(REPO, 'kicad_files',
                                       os.path.splitext(BOARDS[board_kind])[0]
                                       + '.kicad_pro')
                c_rules, c_cls = cli_floors(src_pro)
            diff = {k: (g_rules.get(k), c_rules.get(k))
                    for k in set(g_rules) | set(c_rules)
                    if g_rules.get(k) is None or c_rules.get(k) is None
                    or abs(g_rules[k] - c_rules[k]) > 1e-6}
            check("every rule the two fronts declare is the same", not diff,
                  f"(GUI, CLI): {diff}")
            cdiff = {k: (g_cls.get(k), c_cls.get(k))
                     for k in set(g_cls) | set(c_cls)
                     if g_cls.get(k) is None or c_cls.get(k) is None
                     or abs(g_cls[k] - c_cls[k]) > 1e-6}
            check("and every net class's clearance", not cdiff,
                  f"(GUI, CLI): {cdiff}")

    print("-- the configs the tab really applies with --")
    # The arms above hand _apply_fanout_results this file's CFG. The tab
    # builds its own, one dict literal per fanout kind, and a key missing
    # there is a floor the real run never writes: the QFN dict carried no
    # clearance at all, so the tab wrote no clearance where qfn_fanout does.
    import ast
    need = {'clearance', 'track_width', 'via_size', 'via_drill',
            'fix_drc_settings', 'same_net_pad_clearance', 'class_ceiling'}
    tree = ast.parse(open(os.path.join(REPO, 'kicad_routing_plugin',
                                       'fanout_gui.py'), encoding='utf-8').read())
    lacking = []
    for node in ast.walk(tree):
        if isinstance(node, ast.keyword) and node.arg == 'fanout_config' \
                and isinstance(node.value, ast.Dict):
            keys = {k.value for k in node.value.keys
                    if isinstance(k, ast.Constant)}
            if need - keys:
                lacking.append((node.value.lineno, sorted(need - keys)))
    check("every fanout_config the tab builds carries what the writeback "
          "reads", not lacking, f"{lacking}")

    for d in tmpdirs:
        shutil.rmtree(d, ignore_errors=True)
    print()
    if failures:
        print(f"FAILED ({len(failures)}): " + ", ".join(failures))
        return 1
    print("PASS: the fanout tab writes the floors the CLI fanouts write, on a "
          "project and a project-less board")
    return 0


if __name__ == '__main__':
    sys.exit(main())
