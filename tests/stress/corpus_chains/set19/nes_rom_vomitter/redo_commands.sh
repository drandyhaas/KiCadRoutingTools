#!/bin/bash
# Auto-recorded stress-test command manifest (issue #132).
# Replay with redo_stress_test.py.
set -e
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set19/nes_rom_vomitter
python3 -X utf8 /Users/andy/Documents/KiCadRoutingTools/py_router/route_planes.py --help
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set19/nes_rom_vomitter
python3 -X utf8 /Users/andy/Documents/KiCadRoutingTools/py_router/route_diff.py --help
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set19/nes_rom_vomitter
python3 -X utf8 /Users/andy/Documents/KiCadRoutingTools/py_router/route_planes.py /Users/andy/Documents/kicad_stress_test/boards_unrouted_set19/nes_rom_vomitter.kicad_pcb step1_planes.kicad_pcb --nets GND GND +3V3 --plane-layers In1.Cu In4.Cu In2.Cu --clearance-ceiling 0.15 --hole-to-hole-clearance 0.2 --via-size 0.45 --via-drill 0.3
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set19/nes_rom_vomitter
python3 -X utf8 /Users/andy/Documents/KiCadRoutingTools/py_router/route_diff.py step1_planes.kicad_pcb step2_diff.kicad_pcb --nets 'USB_D*' --track-width 0.1 --diff-pair-gap 0.1 --clearance-ceiling 0.1 --via-size 0.45 --via-drill 0.3 --no-gnd-vias
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set19/nes_rom_vomitter
python3 -X utf8 /Users/andy/Documents/KiCadRoutingTools/py_router/route.py --help
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set19/nes_rom_vomitter
python3 -X utf8 /Users/andy/Documents/KiCadRoutingTools/py_router/route.py step2_diff.kicad_pcb step3_route.kicad_pcb --nets '*' --layers F.Cu In1.Cu In2.Cu In3.Cu In4.Cu B.Cu --clearance-ceiling 0.15 --hole-to-hole-clearance 0.2 --track-width 0.2 --via-size 0.45 --via-drill 0.3 --power-nets GND +3V3 VBUS +5V NES_5V --power-nets-widths 0.3 0.3 0.4 0.4 0.4 --max-ripup 5
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set19/nes_rom_vomitter
python3 -X utf8 /Users/andy/Documents/KiCadRoutingTools/py_router/route.py step2_diff.kicad_pcb step3_route.kicad_pcb --nets '*' --layers F.Cu In1.Cu In2.Cu In3.Cu In4.Cu B.Cu --clearance-ceiling 0.15 --hole-to-hole-clearance 0.2 --track-width 0.2 --via-size 0.45 --via-drill 0.3 --power-nets GND +3V3 VBUS +5V NES_5V --power-nets-widths 0.3 0.3 0.4 0.4 0.4 --max-ripup 5
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set19/nes_rom_vomitter
python3 -X utf8 /Users/andy/Documents/KiCadRoutingTools/py_router/route.py step3_route.kicad_pcb step3b_retry.kicad_pcb --nets NES5V_SENSE NES_5V GND +3V3 --layers F.Cu In1.Cu In2.Cu In3.Cu In4.Cu B.Cu --grid-step 0.05 --clearance-ceiling 0.1 --hole-to-hole-clearance 0.2 --track-width 0.15 --via-size 0.45 --via-drill 0.3 --power-nets GND +3V3 VBUS +5V NES_5V --power-nets-widths 0.3 0.3 0.4 0.4 0.4 --max-ripup 5
