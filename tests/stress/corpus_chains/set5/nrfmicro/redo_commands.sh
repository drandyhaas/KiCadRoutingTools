#!/bin/bash
# Auto-recorded stress-test command manifest (issue #132).
# Replay with redo_stress_test.py.
set -e
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set5/nrfmicro
python3 -u -X utf8 /Users/andy/Documents/KiCadRoutingTools/py_router/route.py --help
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set5/nrfmicro
python3 -u -X utf8 /Users/andy/Documents/KiCadRoutingTools/py_router/route_planes.py --help
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set5/nrfmicro
python3 -u -X utf8 /Users/andy/Documents/KiCadRoutingTools/py_router/route_planes.py /Users/andy/Documents/kicad_stress_test/boards_unrouted_set5/nrfmicro.kicad_pcb step1_planes.kicad_pcb --nets GND --plane-layers B.Cu --clearance-ceiling 0.2 --zone-clearance 0.2 --via-size 0.6 --via-drill 0.33 --track-width 0.2 --hole-to-hole-clearance 0.25 --power-nets GND VBUS VBAT nRF_VDD EXT_VCC --power-nets-widths GND=0.4 VBUS=0.5 VBAT=0.4 nRF_VDD=0.3 EXT_VCC=0.3
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set5/nrfmicro
python3 -u -X utf8 /Users/andy/Documents/KiCadRoutingTools/py_router/route.py --help
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set5/nrfmicro
python3 -u -X utf8 /Users/andy/Documents/KiCadRoutingTools/py_router/route_planes.py /Users/andy/Documents/kicad_stress_test/boards_unrouted_set5/nrfmicro.kicad_pcb step1_planes.kicad_pcb --nets GND --plane-layers B.Cu --clearance-ceiling 0.2 --zone-clearance 0.2 --via-size 0.6 --via-drill 0.33 --track-width 0.2 --hole-to-hole-clearance 0.25 --power-nets GND VBUS VBAT nRF_VDD EXT_VCC --power-nets-widths 0.4 0.5 0.4 0.3 0.3
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set5/nrfmicro
python3 -u -X utf8 /Users/andy/Documents/KiCadRoutingTools/py_router/route_diff.py --help
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set5/nrfmicro
python3 -u -X utf8 /Users/andy/Documents/KiCadRoutingTools/py_router/route_diff.py step1_planes.kicad_pcb step2_diffpair.kicad_pcb --nets DATA+ DATA- --track-width 0.1 --diff-pair-gap 0.1 --clearance-ceiling 0.1 --via-size 0.6 --via-drill 0.33 --hole-to-hole-clearance 0.25 --no-gnd-vias
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set5/nrfmicro
python3 -u -X utf8 /Users/andy/Documents/KiCadRoutingTools/py_router/route.py step2_diffpair.kicad_pcb final_board.kicad_pcb --nets '*' --clearance-ceiling 0.2 --track-width 0.2 --via-size 0.6 --via-drill 0.33 --hole-to-hole-clearance 0.25 --power-nets GND VBUS VBAT nRF_VDD EXT_VCC --power-nets-widths 0.4 0.5 0.4 0.3 0.3
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set5/nrfmicro
python3 -u -X utf8 /Users/andy/Documents/KiCadRoutingTools/py_router/route.py final_board.kicad_pcb final_board2.kicad_pcb --nets '*' --clearance-ceiling 0.2 --track-width 0.1 --via-size 0.6 --via-drill 0.33 --hole-to-hole-clearance 0.25 --power-nets GND VBUS VBAT nRF_VDD EXT_VCC --power-nets-widths 0.4 0.5 0.4 0.3 0.3
