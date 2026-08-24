"""KiCad 10 SWIG ActionPlugin entry point for selected-track gloss."""

from __future__ import annotations

import logging
import os

import pcbnew

from .board_adapter import BoardAdapter
from .gloss_engine import generate_candidate_plans


PLUGIN_DIR = os.path.dirname(os.path.abspath(__file__))
LOG = logging.getLogger("KiCadTrackGloss")

# One-click policy: no dialog, preview, temporary board, DRC subprocess, or
# success/no-op popup. KiCad's native rules still constrain candidate search.
MIN_GAIN_MM = 0.01
ALLOW_EQUAL_LENGTH_SIMPLIFICATION = True


class KiCadTrackGlossPlugin(pcbnew.ActionPlugin):
    def defaults(self):
        self.name = "KiCad Track Gloss"
        self.category = "Routing"
        self.description = ("Shorten and simplify selected PCB track chains "
                            "while preserving connectivity and design rules")
        self.show_toolbar_button = True
        self.icon_file_name = os.path.join(PLUGIN_DIR, "icon_24.png")
        dark = os.path.join(PLUGIN_DIR, "icon_24_dark.png")
        if os.path.exists(dark):
            self.dark_icon_file_name = dark

    def Run(self):
        try:
            self._run()
        except Exception:
            LOG.exception("Track gloss failed; the board was left unchanged")

    def _run(self):
        board = pcbnew.GetBoard()
        if board is None:
            return
        adapter = BoardAdapter(pcbnew)
        try:
            snapshot = adapter.snapshot(board)
        except ValueError:
            return
        if len(snapshot.eligible_keys) < 2:
            return

        plans = generate_candidate_plans(
            snapshot.model, snapshot.eligible_keys,
            min_gain=MIN_GAIN_MM,
            allow_equal_length_simpler=ALLOW_EQUAL_LENGTH_SIMPLIFICATION,
            clearance=snapshot.minimum_clearance)
        plans = [plan for plan in plans if plan.changed]
        if not plans:
            return
        adapter.apply(board, plans[0], rollback_on_error=True)
        try:
            board.SetModified()
        except Exception:
            pass
        pcbnew.Refresh()
