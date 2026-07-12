"""enable_used_layers(): a layer the board uses but that's missing from its
(layers) table gets re-added (canonical id + type), so KiCad shows it as
selectable and stops flagging item_on_disabled_layer. Format-preserving
.kicad_pcb edit (issue: used-but-disabled layers)."""
import os
import re
import shutil
import sys
import tempfile
import unittest

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, ROOT)

from fix_kicad_drc_settings import enable_used_layers, _canonical_layer  # noqa: E402
from kicad_parser import parse_kicad_pcb  # noqa: E402

SRC = os.path.join(ROOT, "kicad_files", "fanout_starting_point.kicad_pcb")


class TestCanonicalLayer(unittest.TestCase):
    def test_known_ids(self):
        self.assertEqual(_canonical_layer("F.Cu"), (0, "signal"))
        self.assertEqual(_canonical_layer("B.Cu"), (2, "signal"))
        self.assertEqual(_canonical_layer("In1.Cu"), (4, "signal"))
        self.assertEqual(_canonical_layer("In8.Cu"), (18, "signal"))
        self.assertEqual(_canonical_layer("F.Fab"), (35, "user"))
        self.assertEqual(_canonical_layer("Edge.Cuts"), (25, "user"))
        self.assertEqual(_canonical_layer("User.1"), (39, "user"))
        self.assertEqual(_canonical_layer("User.9"), (55, "user"))

    def test_unknown_returns_none(self):
        self.assertIsNone(_canonical_layer("User.40"))
        self.assertIsNone(_canonical_layer("In31.Cu"))
        self.assertIsNone(_canonical_layer("not.a.layer"))

    def test_ids_match_sample_board_table(self):
        # Every (id "name" type) in a real board must agree with the canonical map.
        txt = open(SRC).read()
        block = re.search(r'\(layers\b.*?\n[ \t]*\)', txt, re.DOTALL).group(0)
        for lid, name, ltype in re.findall(r'\((\d+)\s+"([^"]+)"\s+(\w+)', block):
            canon = _canonical_layer(name)
            if canon is not None:  # exotic layers we don't map are skipped, not wrong
                self.assertEqual(canon, (int(lid), ltype), name)


class TestEnableUsedLayers(unittest.TestCase):
    def setUp(self):
        self.tmp = tempfile.mkdtemp()
        self.work = os.path.join(self.tmp, "b.kicad_pcb")

    def tearDown(self):
        shutil.rmtree(self.tmp, ignore_errors=True)

    def _board_with_disabled_used_layer(self, layer="F.Fab", lid=35):
        """Copy the sample board, add a gr_text on `layer`, delete its table entry."""
        txt = open(SRC).read()
        txt = txt.replace(
            "(layers",
            '(gr_text "x" (at 10 10) (layer "%s") (uuid "dead-0"))\n\t(layers' % layer, 1)
        txt2 = re.sub(r'\n[ \t]*\(%d "%s" \w+\)' % (lid, re.escape(layer)), "", txt)
        assert txt2 != txt, "fixture did not remove the layer entry"
        open(self.work, "w").write(txt2)

    def test_readds_used_but_disabled_layer(self):
        self._board_with_disabled_used_layer("F.Fab", 35)
        added = enable_used_layers(self.work, verbose=False)
        self.assertEqual(added, ["F.Fab"])
        pcb = parse_kicad_pcb(self.work)
        self.assertEqual(pcb.board_info.layers.get(35), "F.Fab")

    def test_idempotent(self):
        self._board_with_disabled_used_layer("F.Fab", 35)
        enable_used_layers(self.work, verbose=False)
        self.assertEqual(enable_used_layers(self.work, verbose=False), [])

    def test_no_spurious_adds_on_pristine_board(self):
        shutil.copy(SRC, self.work)
        self.assertEqual(enable_used_layers(self.work, verbose=False), [])

    def test_ignores_wildcards_and_stackup(self):
        # Pads carry (layers "*.Cu" "*.Mask" ...) and the stackup carries
        # (layer "dielectric N") — neither should ever be added as a layer.
        shutil.copy(SRC, self.work)
        enable_used_layers(self.work, verbose=False)
        block = re.search(r'\(layers\b.*?\n[ \t]*\)', open(self.work).read(), re.DOTALL).group(0)
        self.assertNotIn("*", block)
        self.assertNotIn("dielectric", block)


if __name__ == "__main__":
    unittest.main()
