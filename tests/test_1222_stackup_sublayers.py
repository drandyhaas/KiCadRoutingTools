#!/usr/bin/env python3
"""#1222: a dielectric made of several sheets counts every sheet.

KiCad writes a core/prepreg built from several sheets (JLCPCB's 3 x 2116) as
ONE `(layer "dielectric N" ...)` whose sheets are joined by `addsublayer`.
`extract_stackup` read the first `(thickness ...)` of the block, so such a
prepreg counted one sheet: on JLC08161H-3313 the F.Cu->B.Cu via barrel read
1.0928 mm of a 1.5736 mm stackup, and a pair on In2 sized for 0.1164 mm of
height instead of 0.3568 mm. The same reader missed `(thickness X locked)`
outright, so a locked dielectric read 0 mm -- librevna's whole stackup read
0.13 mm of copper.

Checks:
  1. The issue's 8-layer stackup: every sheet counts (thickness, barrel,
     impedance height).
  2. A locked thickness is read.
  3. A single-sheet layer comes back exactly as written.
  4. Sheets combine in SERIES: epsilon_r sum(t)/sum(t/er), loss tangent
     weighted by t/er, materials named.
  5. A sheet KiCad left at its sublayer defaults (epsilon_r 1, loss tangent
     0 -- stm32h7_hdmi, jetson_orin) sits out of the averages.
  6. KiCad 8+'s multi-line form, and a stackup with no `copper_finish`, read
     every layer.
  7. The GUI's file fallback reads on when the stackup runs past its 8192-byte
     head.
  8. The GUI's SWIG branch asks for each sublayer.

    python3 tests/test_1222_stackup_sublayers.py
"""
import contextlib
import io
import os
import sys
import tempfile

TESTS_DIR = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.dirname(TESTS_DIR)
sys.path.insert(0, os.path.join(ROOT, 'py_router'))

from kicad_parser import (extract_stackup, _extract_stackup_from_pcbnew,  # noqa: E402
                          PCBData, BoardInfo, StackupLayer)
from impedance import calculate_impedance_for_layer                      # noqa: E402

failures = []


def check(label, ok, detail=''):
    print(f"  [{'ok' if ok else 'FAIL'}] {label}{(': ' + detail) if detail else ''}")
    if not ok:
        failures.append(label)


def close(a, b, tol=1e-9):
    return abs(a - b) <= tol


def layer(stackup, name):
    return next(l for l in stackup if l.name == name)


# JLCPCB JLC08161H-3313: copper thickness, or dielectric kind + each sheet's
# thickness (the issue's reproduction).
JLC = [("F.Cu", .035), ("prepreg", .0994), ("In1.Cu", .0152), ("core", .1),
       ("In2.Cu", .0152), ("prepreg", .1164, .124, .1164), ("In3.Cu", .0152),
       ("core", .3), ("In4.Cu", .0152), ("prepreg", .1164, .124, .1164),
       ("In5.Cu", .0152), ("core", .1), ("In6.Cu", .0152), ("prepreg", .0994),
       ("B.Cu", .035)]
SHEET = '(thickness %s) (material "x") (epsilon_r 4.16) (loss_tangent 0.02)'


def jlc_text():
    text = "(stackup\n"
    for i, (kind, *t) in enumerate(JLC):
        if kind.endswith(".Cu"):
            text += '(layer "%s" (type "copper") (thickness %s))\n' % (kind, t[0])
        else:
            text += '(layer "dielectric %d" (type "%s") %s)\n' % (
                i, kind, " addsublayer ".join(SHEET % x for x in t))
    return text + '(copper_finish "ENIG")\n)\n'


def one(sheets, kind='prepreg'):
    """The single dielectric of a stackup whose sheets are `sheets`."""
    got = extract_stackup('(stackup\n(layer "dielectric 1" (type "%s") %s)\n'
                          '(copper_finish "None")\n)\n' % (kind, " addsublayer ".join(sheets)))
    if len(got) != 1:
        check("the fixture parses to one layer", False, f"{len(got)} layers")
        return StackupLayer('missing', '', 0.0)
    return got[0]


print("1. every sheet counts (the issue's JLC08161H-3313)")
stackup = extract_stackup(jlc_text())
check("dielectric 5 is 0.1164 + 0.124 + 0.1164",
      close(layer(stackup, "dielectric 5").thickness, 0.3568),
      f"{layer(stackup, 'dielectric 5').thickness}")
names = [s[0] for s in JLC if s[0].endswith(".Cu")]
pcb = PCBData(board_info=BoardInfo(layers=dict(enumerate(names)), copper_layers=names,
                                   stackup=stackup),
              nets={}, footprints={}, vias=[], segments=[], pads_by_net={})
barrel = pcb.get_via_barrel_length("F.Cu", "B.Cu")
check("F.Cu -> B.Cu barrel is the whole stackup", close(barrel, 1.5736),
      f"{barrel:.4f} mm")
with contextlib.redirect_stdout(io.StringIO()):
    imp = calculate_impedance_for_layer(pcb, "In2.Cu", 0.115, 0.16)
check("In2.Cu's height below is the whole prepreg",
      close(imp["params"].height_below, 0.3568), f"{imp['params'].height_below}")
check("the sheets share epsilon_r and loss tangent, so both are kept",
      close(layer(stackup, "dielectric 5").epsilon_r, 4.16)
      and close(layer(stackup, "dielectric 5").loss_tangent, 0.02))

print("2. a locked thickness is read (librevna)")
l2 = one(['(color "FR4 natural") (thickness 0.2104 locked) (material "7628") '
          '(epsilon_r 4.4) (loss_tangent 0.02)'])
check("(thickness 0.2104 locked) reads 0.2104", l2.thickness == 0.2104, f"{l2.thickness}")
l2b = one(['(thickness 0.1164 locked) (material "2116") (epsilon_r 4.16)',
           '(thickness 0.124) (material "2116") (epsilon_r 4.16)'])
check("a locked sheet beside an unlocked one", close(l2b.thickness, 0.2404),
      f"{l2b.thickness}")

print("3. a single sheet comes back exactly as written")
l3 = one(['(thickness 0.2) (material "IS400") (epsilon_r 3.9) (loss_tangent 0.022)'],
         kind='core')
check("values untouched", (l3.layer_type, l3.thickness, l3.epsilon_r, l3.loss_tangent,
                           l3.material) == ('core', 0.2, 3.9, 0.022, 'IS400'),
      f"{l3}")

print("4. sheets combine in series")
l4 = one(['(thickness 0.1) (material "1080") (epsilon_r 3.9) (loss_tangent 0.01)',
          '(thickness 0.2) (material "7628") (epsilon_r 4.4) (loss_tangent 0.03)'])
w1, w2 = 0.1 / 3.9, 0.2 / 4.4
check("thickness sums", close(l4.thickness, 0.3), f"{l4.thickness}")
check("epsilon_r is sum(t)/sum(t/er)", close(l4.epsilon_r, 0.3 / (w1 + w2)),
      f"{l4.epsilon_r:.6f}")
check("loss tangent is weighted by t/er",
      close(l4.loss_tangent, (w1 * 0.01 + w2 * 0.03) / (w1 + w2)), f"{l4.loss_tangent:.6f}")
check("differing materials are all named", l4.material == "1080 + 7628", l4.material)
check("one material is named once",
      one(['(thickness 0.12) (material "PR2116") (epsilon_r 4.3)'] * 2).material == "PR2116")

print("5. KiCad's sublayer defaults sit out of the averages")
l5 = one(['(thickness 0.109) (material "2116") (epsilon_r 4.16) (loss_tangent 0)',
          '(thickness 0.218) (material "7628") (epsilon_r 4.16) (loss_tangent 0)',
          '(thickness 0.109) (material "2116") (epsilon_r 1) (loss_tangent 0)'])
check("stm32h7_hdmi: (epsilon_r 1) does not drag 4.16 toward vacuum",
      close(l5.epsilon_r, 4.16), f"{l5.epsilon_r}")
check("stm32h7_hdmi: the sheet's thickness still counts", close(l5.thickness, 0.436),
      f"{l5.thickness}")
l5b = one(['(thickness 0.12) (material "PR2116") (epsilon_r 4.39) (loss_tangent 0.02)',
           '(thickness 0.12) (material "PR2116") (epsilon_r 4.39) (loss_tangent 0)'])
check("jetson_orin: an unfilled (loss_tangent 0) does not halve 0.02",
      close(l5b.loss_tangent, 0.02), f"{l5b.loss_tangent}")
l5c = one(['(thickness 0.1)', '(thickness 0.1)'])
check("no sheet declares epsilon_r: 0, as for any undeclared layer",
      l5c.epsilon_r == 0.0 and close(l5c.thickness, 0.2), f"{l5c}")

print("6. KiCad 8+'s multi-line form, with and without copper_finish")
PRETTY = '''\t\t(stackup
\t\t\t(layer "F.SilkS"
\t\t\t\t(type "Top Silk Screen")
\t\t\t)
\t\t\t(layer "F.Cu"
\t\t\t\t(type "copper")
\t\t\t\t(thickness 0.035)
\t\t\t)
\t\t\t(layer "dielectric 1"
\t\t\t\t(type "prepreg")
\t\t\t\t(thickness 0.12)
\t\t\t\t(material "PR2116")
\t\t\t\t(epsilon_r 4.3)
\t\t\t\t(loss_tangent 0.014) addsublayer
\t\t\t\t(thickness 0.12)
\t\t\t\t(material "PR2116")
\t\t\t\t(epsilon_r 4.3)
\t\t\t\t(loss_tangent 0.014)
\t\t\t)
\t\t\t(layer "B.Cu"
\t\t\t\t(type "copper")
\t\t\t\t(thickness 0.035)
\t\t\t)
\t\t\t(layer "B.SilkS"
\t\t\t\t(type "Bottom Silk Screen")
\t\t\t)
%s\t\t\t(dielectric_constraints no)
\t\t)
\t\t(pad_to_mask_clearance 0)
'''
for label, finish in (("with copper_finish", '\t\t\t(copper_finish "ENIG")\n'),
                      ("without copper_finish", '')):
    s6 = extract_stackup(PRETTY % finish)
    check(label, [(l.name, round(l.thickness, 6)) for l in s6]
          == [("F.Cu", 0.035), ("dielectric 1", 0.24), ("B.Cu", 0.035)],
          f"{[(l.name, l.thickness) for l in s6]}")


class _NoSwigStackup:
    """A pcbnew board whose stackup descriptor has no GetList (KiCad 10)."""
    def __init__(self, path):
        self._path = path

    def GetDesignSettings(self):
        raise AttributeError("GetStackupDescriptor().GetList")

    def GetFileName(self):
        return self._path


print("7. the GUI's file fallback reads past its head")
HEAD = 8192
with tempfile.TemporaryDirectory() as tmp:
    path = os.path.join(tmp, 'deep.kicad_pcb')
    for label, pad_len, starts_in_head in (("runs out of the head", 7500, True),
                                           ("starts past the head", 9000, False)):
        pad = '(title_block (comment 1 "%s"))\n' % ('x' * pad_len)
        body = '(kicad_pcb (version 20241229)\n%s(setup\n%s)\n)\n' % (pad, jlc_text())
        with open(path, 'w', encoding='utf-8') as f:
            f.write(body)
        start, end = body.index('(stackup'), body.index('(copper_finish')
        check(f"precondition ({label})",
              (start < HEAD) == starts_in_head and end > HEAD, f"stackup at {start}..{end}")
        gui = _extract_stackup_from_pcbnew(_NoSwigStackup(path), lambda v: v / 1e6)
        text = extract_stackup(body)
        check(f"{label}: GUI fallback == text parse",
              [vars(l) for l in gui] == [vars(l) for l in text],
              f"{len(gui)} vs {len(text)} layers")
        check(f"{label}: ...and the text parse is whole", len(text) == len(JLC),
              f"{len(text)} layers")
    # The common case: the stackup fits in the head.
    with open(path, 'w', encoding='utf-8') as f:
        f.write('(kicad_pcb (version 20241229)\n(setup\n%s)\n)\n' % jlc_text())
    gui = _extract_stackup_from_pcbnew(_NoSwigStackup(path), lambda v: v / 1e6)
    check("a stackup inside the head still parses", len(gui) == len(JLC), f"{len(gui)}")


class _Item:
    """A BOARD_STACKUP_ITEM as pcbnew's SWIG would expose it: one getter per
    property, indexed by sublayer, defaulting to the first."""
    def __init__(self, name, kind, sheets):
        self._name, self._kind, self._sheets = name, kind, sheets

    def GetLayerName(self):
        return self._name

    def GetTypeName(self):
        return self._kind

    def GetSublayersCount(self):
        return len(self._sheets)

    def GetThickness(self, k=0):
        return int(round(self._sheets[k][0] * 1e6))

    def GetEpsilonR(self, k=0):
        return self._sheets[k][1]

    def GetLossTangent(self, k=0):
        return self._sheets[k][2]

    def GetMaterial(self, k=0):
        return self._sheets[k][3]


class _SwigBoard:
    def __init__(self, items):
        self._items = items

    def GetDesignSettings(self):
        board = self

        class _DS:
            def GetStackupDescriptor(self):
                class _Desc:
                    def GetList(self):
                        return board._items
                return _Desc()
        return _DS()

    def GetFileName(self):
        return ''


print("8. the GUI's SWIG branch asks for each sublayer")
items = [_Item("F.Cu", "copper", [(0.035, 0.0, 0.0, "")]),
         _Item("dielectric 1", "prepreg", [(0.1164, 4.16, 0.02, "x"), (0.124, 4.16, 0.02, "x"),
                                           (0.1164, 4.16, 0.02, "x")]),
         _Item("B.Cu", "copper", [(0.035, 0.0, 0.0, "")])]
swig = _extract_stackup_from_pcbnew(_SwigBoard(items), lambda v: v / 1e6)
check("three sheets read through the indexed getters",
      len(swig) == 3 and close(swig[1].thickness, 0.3568), f"{[l.thickness for l in swig]}")
text = extract_stackup('(stackup\n(layer "F.Cu" (type "copper") (thickness 0.035))\n'
                       '(layer "dielectric 1" (type "prepreg") %s)\n'
                       '(layer "B.Cu" (type "copper") (thickness 0.035))\n'
                       '(copper_finish "None")\n)\n'
                       % " addsublayer ".join(SHEET % t for t in (.1164, .124, .1164)))
check("SWIG branch == text parse", [vars(l) for l in swig] == [vars(l) for l in text])

if failures:
    print(f"\nFAILED: {failures}")
    sys.exit(1)
print("\nALL PASSED")
