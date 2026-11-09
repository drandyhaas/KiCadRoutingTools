"""Does the REAL dialog resolve plane track_width from the board's Default
netclass (CLI parity), or fall back to defaults.TRACK_WIDTH like the shim?"""

# ---------------------------------------------------------------------------
# macOS: if this HANGS at ~0% CPU, it is NOT wx, machine load, or a deadlock.
#
# After any wx process here is killed (a pkill, a timeout, a crash), macOS
# decides the app "quit unexpectedly", and the NEXT headless launch stops inside
# NSApplication bootstrap showing the restore-windows alert you cannot see:
#     -[NSPersistentUIRestorer promptToIgnorePersistentState]
#         -> -[NSAlert runModal]
# Headless, nobody can click it, so it waits forever: process state SN accruing
# ~0.3s of CPU over many minutes, which reads exactly like a hang. This cost a
# full session of ".gui-parity-checked" markers recording "wx blocked, gate NOT
# RUN" -- the gates were fine the whole time.
#
#   diagnose:  sample <pid> 3 -mayDie | grep -E "NSAlert|PersistentUI"
#   fix:       defaults write -g ApplePersistenceIgnoreState -bool YES
#
# A sandboxed HOME does NOT help -- cfprefsd serves that pref per-user
# regardless of HOME. With the default set, test_gui_engine_parity.py runs ~90s.
# ---------------------------------------------------------------------------
import os, sys
os.environ.setdefault('WXSUPPRESS_SIZER_FLAGS_CHECK', '1')
# Repo root from THIS file's location (tests/gui_parity/ -> repo), never a
# hardcoded home dir -- the sibling gates all derive it this way, and an
# absolute path silently probed the MAIN checkout from inside a worktree.
# (Both branches fixed this independently; same derivation.) This branch also
# puts the gate's own directory on sys.path for `fake_ipc_board`, and imports no
# pcbnew -- the IPC plugin reaches the board over kipy instead.
_HERE = os.path.dirname(os.path.abspath(__file__))
R = os.path.dirname(os.path.dirname(_HERE))
sys.path.insert(0, R); sys.path.insert(0, _HERE)
import wx, json
import routing_defaults as defaults
import kicad_parser
from kicad_routing_plugin import routing_dialog

board_path = sys.argv[1]
app = wx.App(False)
wx.Log.SetActiveTarget(wx.LogStderr())
# IPC: no in-process board -- back the plugin's "live board" with the file.
from fake_ipc_board import install as _install_fake_board
board = _install_fake_board(board_path)
# Call through the MODULE, not a name bound at import time: install() rebinds
# kicad_parser.build_pcb_data_from_board, and a `from ... import` above would
# have captured the original and driven the real kipy reader at the fake board.
pd = kicad_parser.build_pcb_data_from_board(board)
dlg = routing_dialog.RoutingDialog(None, pd, board_path)

pro = os.path.splitext(board_path)[0] + '.kicad_pro'
proj = None
if os.path.exists(pro):
    for c in json.load(open(pro)).get('net_settings', {}).get('classes', []):
        if c.get('name') == 'Default':
            proj = c.get('track_width')
print(f'board .kicad_pro Default track_width : {proj}')
print(f'defaults.TRACK_WIDTH (shim fallback) : {defaults.TRACK_WIDTH}')
print(f'track_width override checkbox        : {dlg.track_width_check.GetValue()}')
print(f'raw control value                    : {dlg.track_width.GetValue()}')
print(f'_effective_track_width()             : {dlg._effective_track_width()}')
print(f'planes tab shared track_width        : {dlg.planes_tab.get_shared_params().get("track_width")}')
