#!/usr/bin/env python3
"""Round-trip candidate boards through KiCad 10 pcbnew to normalize format.

Run with KiCad's bundled Python:
/Applications/KiCad/KiCad.app/Contents/Frameworks/Python.framework/Versions/Current/bin/python3
"""
import sys
from pathlib import Path
import os
STRESS = Path(os.environ.get("STRESS_DIR", str(Path.home() / "Documents/kicad_stress_test")))
import pcbnew

SRC = STRESS / "sources/github_set1"
DST = STRESS / "boards_set1"
DST.mkdir(parents=True, exist_ok=True)

# curated: source filename fragment -> short board name
BOARDS = {
    "beekeeb__piantor__pcb_right_keyboard_pcb": "piantor",
    "duckyb__urchin__main": "urchin",
    "gcormier__megadesk__pcb_megadesk": "megadesk",
    "tjhorner__upsy-desky__pcb_upsy-desky": "upsy_desky",
    "cardonabits__haxo-hw__haxophone001": "haxophone",
    "scottbez1__smartknob__electronics_view_base_view_base": "smartknob_base",
    "scottbez1__splitflap__electronics_chainlinkDriver_chainlinkDriver": "splitflap_driver",
    "skot__bitaxe__bitaxeUltra": "bitaxe_ultra",
    "Ottercast__OtterCastAudioV2__OtterCastAudioV2": "ottercast_audio",
    "wntrblm__Castor_and_Pollux__hardware_mainboard_mainboard": "castor_pollux",
    "opulo-inc__lumenpnp__pnp_pcb_mobo_mobo.kicad_pcb": "lumenpnp_mobo",
    "GlasgowEmbedded__glasgow__hardware_boards_glasgow_glasgow.kicad_pcb": "glasgow_revC",
    "greatscottgadgets__hackrf__hardware_hackrf-one_hackrf-one": "hackrf_one",
    "OLIMEX__Neo6502__HARDWARE_Neo6502-rev-B1_Neo6502_Rev_B1": "neo6502",
}

for frag, name in BOARDS.items():
    matches = [f for f in SRC.glob("*.kicad_pcb") if frag in f.name]
    if not matches:
        print(f"MISS {name}: no file matching '{frag}'")
        continue
    src = matches[0]
    dst = DST / f"{name}.kicad_pcb"
    try:
        board = pcbnew.LoadBoard(str(src))
        pcbnew.SaveBoard(str(dst), board)
        print(f"OK   {name}  <- {src.name[:70]}")
    except Exception as e:
        print(f"FAIL {name}: {type(e).__name__}: {e}")
